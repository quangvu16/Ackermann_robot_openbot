//
//  Created by Sparsh Jain on 22/07/22.
//

import UIKit
import DropDown
import Firebase
import FirebaseAuth
import GoogleSignIn
@main
class AppDelegate: UIResponder, UIApplicationDelegate {
    var restrictRotation: UIInterfaceOrientationMask = .all

    var orientationLock = UIInterfaceOrientationMask.all

    func application(_ application: UIApplication, supportedInterfaceOrientationsFor window: UIWindow?) -> UIInterfaceOrientationMask {
        orientationLock
    }

    struct AppUtility {
        static func lockOrientation(_ orientation: UIInterfaceOrientationMask) {
            if let delegate = UIApplication.shared.delegate as? AppDelegate {
                delegate.orientationLock = orientation
            }
        }

        static func lockOrientation(_ orientation: UIInterfaceOrientationMask, andRotateTo rotateOrientation: UIInterfaceOrientation) {
            self.lockOrientation(orientation)
            UIDevice.current.setValue(rotateOrientation.rawValue, forKey: "orientation")
        }
    }

    func application(_ application: UIApplication, didFinishLaunchingWithOptions launchOptions: [UIApplication.LaunchOptionsKey: Any]?) -> Bool {
        // Override point for customization after application launch.'
        DropDown.startListeningToKeyboard()
        // Firebase (Google sign-in, Drive projects, profile) only starts with a
        // real GoogleService-Info.plist for this bundle ID. With OpenBot's dummy
        // placeholder the app still drives, runs missions and shows the RC faces.
        FirebaseSupport.configureIfAvailable()
        if FirebaseSupport.isConfigured {
            GIDSignIn.sharedInstance.restorePreviousSignIn { user, error in
                if error != nil || user == nil {
                    // The user isn't signed in or there was an error restoring their session.
                } else {
                    // The user is signed in and their session has been restored.
                }
            }
        }
        return true
    }
    
    // MARK: UISceneSession Lifecycle

    func application(_ application: UIApplication, configurationForConnecting connectingSceneSession: UISceneSession, options: UIScene.ConnectionOptions) -> UISceneConfiguration {
        // Called when a new scene session is being created.
        // Use this method to select a configuration to create the new scene with.
        UISceneConfiguration(name: "Default Configuration", sessionRole: connectingSceneSession.role)
    }

    func application(_ application: UIApplication, didDiscardSceneSessions sceneSessions: Set<UISceneSession>) {
        // Called when the user discards a scene session.
        // If any sessions were discarded while the application was not running, this will be called shortly after application:didFinishLaunchingWithOptions.
        // Use this method to release any resources that were specific to the discarded scenes, as they will not return.
    }
    func application(_ app: UIApplication, open url: URL, options: [UIApplication.OpenURLOptionsKey : Any] = [:]) -> Bool {
        return GIDSignIn.sharedInstance.handle(url)
    }

    func sign(_ signIn: GIDSignIn!, didSignInFor user: GIDGoogleUser!, withError error: Error?) {
        if let error = error {
            print("Google Sign-In error: \(error.localizedDescription)")
            return
        }

        // The signed-in user is stored in the user property of the GIDSignIn instance.
        let userIdToken = user.accessToken
        print("Access Token: \(userIdToken )")
    }
}

/// Starts Firebase only when the app has a real config for its own bundle ID,
/// and gives the rest of the app a crash-free way to reach Firebase Auth.
///
/// `Auth.auth()` traps when Firebase was never configured, so every caller goes
/// through `FirebaseSupport.auth`, which is nil in that case ("nobody signed in").
enum FirebaseSupport {
    private(set) static var isConfigured = false

    static func configureIfAvailable() {
        guard let path = Bundle.main.path(forResource: "GoogleService-Info", ofType: "plist"),
              let options = NSDictionary(contentsOfFile: path) as? [String: Any] else {
            print("OpenBot: no GoogleService-Info.plist - Firebase and Google sign-in are off")
            return
        }
        let apiKey = options["API_KEY"] as? String ?? ""
        let projectId = options["PROJECT_ID"] as? String ?? ""
        let bundleId = options["BUNDLE_ID"] as? String ?? ""
        if apiKey.contains("DUMMY") || projectId == "example-project" {
            print("OpenBot: dummy GoogleService-Info.plist - Firebase and Google sign-in are off")
            return
        }
        if bundleId != Bundle.main.bundleIdentifier {
            print("OpenBot: GoogleService-Info.plist is for \(bundleId), not \(Bundle.main.bundleIdentifier ?? "?") - Firebase is off")
            return
        }
        FirebaseApp.configure()
        isConfigured = true
    }

    /// Firebase Auth, or nil when Firebase is off.
    static var auth: Auth? {
        isConfigured ? Auth.auth() : nil
    }
}
