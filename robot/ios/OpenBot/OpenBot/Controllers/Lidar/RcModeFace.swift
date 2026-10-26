//
//  RcModeFace.swift
//  OpenBot
//
//  Full-screen face for the car's RC mode switch. The firmware reports the
//  switch as "m<letter>" on every change and once a second (see
//  firmware/openbot-esp32s3/openbot-esp32s3.cpp, send_rc_mode()):
//
//    MANUAL  the RC transmitter drives  -> smiling face
//    AUTO    the app drives             -> no face
//    HOLD    stopped by the switch      -> crying face
//    LOST    RC link dropped, stopped   -> crying face
//
//  The face covers every screen. Tapping hides it until the mode changes, so
//  the app stays reachable. If reports stop for 3 s (Bluetooth dropped, car
//  off) the face goes away. Mirrors the Android RcModeFaceView.
//

import UIKit

/// The RC switch position reported on the `m` channel.
enum RcMode: Equatable {
    case manual, auto, hold, lost

    enum Face: Equatable { case none, smile, cry }

    /// Parses a whole message such as "mM" or "mH\n". Returns nil for
    /// anything else, so a garbled notification never changes the screen.
    static func parse(_ message: String) -> RcMode? {
        let text = message.trimmingCharacters(in: .whitespacesAndNewlines)
        guard text.count == 2, text.first == "m", let letter = text.last else { return nil }
        switch letter {
        case "M": return .manual
        case "A": return .auto
        case "H": return .hold
        case "L": return .lost
        default: return nil
        }
    }

    /// Smile for MANUAL, cry for HOLD and LOST (the car is stopped), nothing for AUTO.
    var face: Face {
        switch self {
        case .manual: return .smile
        case .hold, .lost: return .cry
        case .auto: return .none
        }
    }
}

/// Shows and hides the face over the app's key window.
final class RcModeFaceOverlay {
    static let shared = RcModeFaceOverlay()
    static let reportTimeout: TimeInterval = 3

    private var view: RcModeFaceView?
    private var mode: RcMode?

    /// The latest RC switch report (nil: none for 3 s), shown on the Car dashboard.
    var currentMode: RcMode? { mode }
    /// The face view while it is on screen, so the STOP button can sit below it.
    var faceView: UIView? { view }
    private var dismissedMode: RcMode?
    private var timeout: DispatchWorkItem?

    /// Feed every message from the car; only `m` reports do anything.
    func handle(message: String) {
        guard let parsed = RcMode.parse(message) else { return }
        DispatchQueue.main.async { self.apply(parsed) }
    }

    private func apply(_ newMode: RcMode?) {
        if newMode != mode { dismissedMode = nil }  // a new mode shows again even after a tap
        mode = newMode

        timeout?.cancel()
        if newMode != nil {
            let work = DispatchWorkItem { [weak self] in self?.apply(nil) }
            timeout = work
            DispatchQueue.main.asyncAfter(deadline: .now() + RcModeFaceOverlay.reportTimeout, execute: work)
        }

        guard let shown = newMode, shown.face != .none, shown != dismissedMode else {
            hide()
            return
        }
        present(shown)
    }

    private func present(_ shown: RcMode) {
        guard let window = Self.keyWindow() else { return }
        let faceView = view ?? RcModeFaceView(frame: window.bounds)
        faceView.frame = window.bounds
        faceView.autoresizingMask = [.flexibleWidth, .flexibleHeight]
        faceView.mode = shown
        faceView.onTap = { [weak self] in
            self?.dismissedMode = self?.mode
            self?.hide()
        }
        if faceView.superview !== window { window.addSubview(faceView) }
        window.bringSubviewToFront(faceView)  // stay above screens presented since
        view = faceView
    }

    private func hide() {
        view?.removeFromSuperview()
        view = nil
    }

    private static func keyWindow() -> UIWindow? {
        let windows = UIApplication.shared.connectedScenes
            .compactMap { $0 as? UIWindowScene }
            .flatMap { $0.windows }
        return windows.first(where: { $0.isKeyWindow }) ?? windows.first
    }
}

/// Draws the face with plain UIBezierPath calls, so it needs no image assets
/// and scales to any screen.
final class RcModeFaceView: UIView {
    var mode: RcMode = .manual { didSet { setNeedsDisplay() } }
    var onTap: (() -> Void)?

    private static let bgSmile = UIColor(red: 0x2E / 255, green: 0x7D / 255, blue: 0x32 / 255, alpha: 1)
    private static let bgCry = UIColor(red: 0x37 / 255, green: 0x47 / 255, blue: 0x4F / 255, alpha: 1)
    private static let skin = UIColor(red: 1, green: 0xD5 / 255, blue: 0x4F / 255, alpha: 1)
    private static let outline = UIColor(red: 0x5D / 255, green: 0x40 / 255, blue: 0x37 / 255, alpha: 1)
    private static let mouth = UIColor(red: 0x8D / 255, green: 0x2B / 255, blue: 0x2B / 255, alpha: 1)
    private static let tear = UIColor(red: 0x42 / 255, green: 0xA5 / 255, blue: 0xF5 / 255, alpha: 1)
    private static let cheek = UIColor(red: 1, green: 0x70 / 255, blue: 0x70 / 255, alpha: 0.4)

    override init(frame: CGRect) {
        super.init(frame: frame)
        isOpaque = true
        contentMode = .redraw
        // Swallows touches, so the screen underneath is not driven by accident.
        addGestureRecognizer(UITapGestureRecognizer(target: self, action: #selector(tapped)))
    }

    required init?(coder: NSCoder) {
        fatalError("init(coder:) is not used")
    }

    @objc private func tapped() {
        onTap?()
    }

    override func draw(_ rect: CGRect) {
        let smile = mode.face == .smile
        let w = bounds.width
        let h = bounds.height
        let r = min(w, h) * 0.32
        let cx = w / 2
        let cy = h * 0.42

        (smile ? Self.bgSmile : Self.bgCry).setFill()
        UIRectFill(bounds)

        // head
        let head = UIBezierPath(arcCenter: CGPoint(x: cx, y: cy), radius: r, startAngle: 0, endAngle: 2 * .pi, clockwise: true)
        Self.skin.setFill()
        head.fill()
        Self.outline.setStroke()
        head.lineWidth = r * 0.05
        head.stroke()

        let eyeDx = r * 0.36
        let eyeY = cy - r * 0.18
        let eyeR = r * 0.16

        if smile {
            // happy closed eyes: upward arcs (pi -> 0 clockwise passes over the top)
            for side: CGFloat in [-1, 1] {
                let eye = UIBezierPath(arcCenter: CGPoint(x: cx + side * eyeDx, y: eyeY + eyeR * 0.4), radius: eyeR, startAngle: .pi, endAngle: 0, clockwise: true)
                eye.lineWidth = r * 0.07
                eye.lineCapStyle = .round
                eye.stroke()
            }
            // rosy cheeks
            Self.cheek.setFill()
            for side: CGFloat in [-1, 1] {
                UIBezierPath(arcCenter: CGPoint(x: cx + side * r * 0.55, y: cy + r * 0.15), radius: r * 0.13, startAngle: 0, endAngle: 2 * .pi, clockwise: true).fill()
            }
            // big open smile: lower half-disc
            let mouth = UIBezierPath(arcCenter: CGPoint(x: cx, y: cy + r * 0.20), radius: r * 0.48, startAngle: 0, endAngle: .pi, clockwise: true)
            mouth.close()
            Self.mouth.setFill()
            mouth.fill()
            mouth.lineWidth = r * 0.05
            mouth.stroke()
        } else {
            // sad eyebrows, high in the middle
            for side: CGFloat in [-1, 1] {
                let ex = cx + side * eyeDx
                let brow = UIBezierPath()
                brow.move(to: CGPoint(x: ex + side * eyeR, y: eyeY - r * 0.20))
                brow.addLine(to: CGPoint(x: ex - side * eyeR, y: eyeY - r * 0.32))
                brow.lineWidth = r * 0.06
                brow.lineCapStyle = .round
                brow.stroke()
            }
            // squeezed eyes: downward arcs (0 -> pi clockwise passes underneath)
            for side: CGFloat in [-1, 1] {
                let eye = UIBezierPath(arcCenter: CGPoint(x: cx + side * eyeDx, y: eyeY - eyeR * 0.4), radius: eyeR, startAngle: 0, endAngle: .pi, clockwise: true)
                eye.lineWidth = r * 0.07
                eye.lineCapStyle = .round
                eye.stroke()
            }
            // streams of tears down both cheeks
            Self.tear.setFill()
            let tearW = r * 0.15
            for side: CGFloat in [-1, 1] {
                let tx = cx + side * eyeDx
                let stream = CGRect(x: tx - tearW / 2, y: eyeY + eyeR * 0.5, width: tearW, height: cy + r * 0.80 - (eyeY + eyeR * 0.5))
                UIBezierPath(roundedRect: stream, cornerRadius: tearW / 2).fill()
            }
            // open crying mouth, turned down: upper half-disc
            let mouth = UIBezierPath(arcCenter: CGPoint(x: cx, y: cy + r * 0.72), radius: r * 0.32, startAngle: .pi, endAngle: 0, clockwise: true)
            mouth.close()
            Self.mouth.setFill()
            mouth.fill()
            Self.outline.setStroke()
            mouth.lineWidth = r * 0.05
            mouth.stroke()
        }

        // caption under the face
        let key: String
        switch mode {
        case .manual: key = "rc_mode_manual"
        case .lost: key = "rc_mode_lost"
        default: key = "rc_mode_hold"
        }
        let centered = NSMutableParagraphStyle()
        centered.alignment = .center
        let textY = cy + r + r * 0.25
        let title = L10n.string(key) as NSString
        title.draw(in: CGRect(x: 0, y: textY, width: w, height: r * 0.35), withAttributes: [
            .font: UIFont.boldSystemFont(ofSize: r * 0.20),
            .foregroundColor: UIColor.white,
            .paragraphStyle: centered,
        ])
        let hint = L10n.string("rc_mode_tap_hint") as NSString
        hint.draw(in: CGRect(x: 0, y: textY + r * 0.34, width: w, height: r * 0.25), withAttributes: [
            .font: UIFont.systemFont(ofSize: r * 0.12),
            .foregroundColor: UIColor.white.withAlphaComponent(0.7),
            .paragraphStyle: centered,
        ])
    }
}
