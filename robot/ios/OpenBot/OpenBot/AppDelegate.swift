//
//  Created by Sparsh Jain on 22/07/22.
//

import UIKit
import DropDown
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
        // Google sign-in (for Google Drive projects) needs a real OAuth client ID.
        // Without one the app still drives, runs missions and shows the RC faces.
        if GoogleAccount.clientID != nil {
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

/// The signed-in Google account, without Firebase.
///
/// Google sign-in is only used for Google Drive projects. Its OAuth client ID
/// comes from `GIDClientID` in Info.plist or `CLIENT_ID` in
/// GoogleService-Info.plist; OpenBot's placeholder (all zeros) counts as none,
/// so sign-in simply stays off instead of failing.
enum GoogleAccount {
    static var clientID: String? {
        let candidates: [String?] = [
            Bundle.main.object(forInfoDictionaryKey: "GIDClientID") as? String,
            Bundle.main.path(forResource: "GoogleService-Info", ofType: "plist")
                .flatMap { NSDictionary(contentsOfFile: $0)?["CLIENT_ID"] as? String },
        ]
        for case let id? in candidates where !id.isEmpty && !id.hasPrefix("000000000000") {
            return id
        }
        return nil
    }

    static var user: GIDGoogleUser? { GIDSignIn.sharedInstance.currentUser }
    static var email: String? { user?.profile?.email }
    static var displayName: String? { user?.profile?.name }
    static func photoURL(dimension: UInt = 100) -> URL? { user?.profile?.imageURL(withDimension: dimension) }
}
