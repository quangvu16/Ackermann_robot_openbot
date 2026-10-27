import UIKit

/// Buttons show an SF Symbol instead of words. The words stay as the VoiceOver label, the pointer
/// tooltip (iPad trackpad) and a small bubble shown on a long press, so nothing is lost.
///
/// Apple HIG, Buttons: "Use an icon ... when it's familiar" and every icon-only button needs an
/// accessibility label (developer.apple.com/design/human-interface-guidelines/buttons).
extension UIButton {

    /// Set the button's SF Symbol and its name. Keeps the rest of the configuration (style, colours).
    func setIcon(_ symbol: String, label: String, pointSize: CGFloat = 20) {
        var c = configuration ?? .plain()
        c.title = nil
        c.attributedTitle = nil
        c.image = UIImage(systemName: symbol)
        c.preferredSymbolConfigurationForImage = UIImage.SymbolConfiguration(pointSize: pointSize, weight: .semibold)
        configuration = c
        accessibilityLabel = label
        toolTip = label
        if !(gestureRecognizers ?? []).contains(where: { $0 is IconHintPress }) {
            addGestureRecognizer(IconHintPress(target: self, action: #selector(showIconHint(_:))))
        }
    }

    /// Convenience for a new icon button.
    static func icon(_ symbol: String, label: String, style: UIButton.Configuration = .bordered(), pointSize: CGFloat = 20) -> UIButton {
        let b = UIButton(configuration: style)
        b.setIcon(symbol, label: label, pointSize: pointSize)
        return b
    }

    @objc private func showIconHint(_ g: UILongPressGestureRecognizer) {
        guard g.state == .began, let text = accessibilityLabel, let host = window else { return }
        IconHint.show(text, above: self, in: host)
    }
}

private final class IconHintPress: UILongPressGestureRecognizer {
    override init(target: Any?, action: Selector?) {
        super.init(target: target, action: action)
        minimumPressDuration = 0.5
        cancelsTouchesInView = true // a long press names the button and does not press it
    }
}

/// The bubble with the button's name (like Android's long-press tooltip).
enum IconHint {
    static func show(_ text: String, above view: UIView, in host: UIView) {
        let label = PaddedLabel()
        label.text = text
        label.font = .systemFont(ofSize: 14, weight: .medium)
        label.textColor = .white
        label.backgroundColor = UIColor.black.withAlphaComponent(0.85)
        label.layer.cornerRadius = 8
        label.clipsToBounds = true
        label.numberOfLines = 2
        label.textAlignment = .center
        let maxW = host.bounds.width - 32
        let size = label.sizeThatFits(CGSize(width: maxW, height: 80))
        let f = view.convert(view.bounds, to: host)
        var x = f.midX - size.width / 2
        x = min(max(16, x), host.bounds.width - 16 - size.width)
        var y = f.minY - size.height - 6
        if y < host.safeAreaInsets.top { y = f.maxY + 6 }
        label.frame = CGRect(x: x, y: y, width: size.width, height: size.height)
        label.alpha = 0
        host.addSubview(label)
        UIImpactFeedbackGenerator(style: .light).impactOccurred()
        UIView.animate(withDuration: 0.15) { label.alpha = 1 }
        UIView.animate(withDuration: 0.3, delay: 1.5, options: []) { label.alpha = 0 } completion: { _ in label.removeFromSuperview() }
    }
}

private final class PaddedLabel: UILabel {
    private let inset = UIEdgeInsets(top: 6, left: 10, bottom: 6, right: 10)
    override func drawText(in rect: CGRect) { super.drawText(in: rect.inset(by: inset)) }
    override func sizeThatFits(_ size: CGSize) -> CGSize {
        let s = super.sizeThatFits(CGSize(width: size.width - inset.left - inset.right, height: size.height))
        return CGSize(width: s.width + inset.left + inset.right, height: s.height + inset.top + inset.bottom)
    }
}

extension UISegmentedControl {
    /// Segments with SF Symbols; each image carries its name for VoiceOver.
    static func icons(_ items: [(symbol: String, label: String)]) -> UISegmentedControl {
        let images: [UIImage] = items.map { item in
            let img = UIImage(systemName: item.symbol) ?? UIImage()
            img.accessibilityLabel = item.label
            return img
        }
        return UISegmentedControl(items: images)
    }
}
