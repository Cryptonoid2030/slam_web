# SLAM Capture (iPhone)

SwiftUI app that captures ARKit LiDAR depth, the camera image and CoreMotion and sends them to the
relay in `../live`. By default it does the corner detection on the phone (`LandmarkExtractor.swift`) and sends
only landmark points, patches and IMU; no images leave the phone. The page can switch it to raw
frames. It does no SLAM itself. Full guide: [docs/guides/live-iphone.md](../docs/guides/live-iphone.md).

Requires an iPhone with LiDAR (12 Pro or newer Pro models) on iOS 16 or later, and Xcode 15 or later.
These sources have not been compiled yet; expect to fix small build errors on the first build.

## Build

```sh
brew install xcodegen
cd ios && xcodegen generate && open SlamCapture.xcodeproj
```

In Xcode: select the **SlamCapture** target → **Signing & Capabilities** → choose your Apple ID as
Team (a free Personal Team works, but the app then expires after 7 days and must be re-installed).
Pick your iPhone and press Run. On the phone, trust the developer profile
(Settings → General → VPN & Device Management) if iOS asks.

Without XcodeGen: create an iOS **App** project (SwiftUI, Swift), delete its generated Swift files,
drag in the Swift files from `SlamCapture/`, and add the Info.plist keys listed in `project.yml`.

## Use

1. Start the relay on the Mac (`npm run live`) and the SLAM tab (see the guide).
2. Enter the Mac's address as `192.168.1.20:8787` (and the token, if you set one) and tap **Start streaming**.
3. Allow Camera, Motion and Local Network access on first launch.
4. Walk slowly. **Show map** opens the relay's mirror page inside the app.

Landmark mode is chosen by the page (**Phone sends**) and arrives as a `config` message; the app
shows what it is sending. With no config within 3 s it falls back to raw frames. Build in Release
or keep the `-O` setting in `project.yml` (also set in the hand-edited project file): the corner
detector is far too slow unoptimised. After editing `project.yml`, rerun `xcodegen generate`.
`LandmarkExtractor.swift` is a port of `src/landmark-source.ts`; it has not been compiled or run, so
the first build may need fixes and its output should be compared with the TypeScript reference.

In raw mode the app sends camera frames at 4 Hz and IMU at 100 Hz, keeps the screen awake while streaming,
and stops when the Mac asks it to (the SLAM fell behind, or a frame gap broke the chain).
