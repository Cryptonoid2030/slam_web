# D435 color and depth with iPhone IMU

The Windows sender streams the D435's color and depth to the main SLAM Lab app. The iPhone
provides Core Motion samples over Wi-Fi. The native estimator builds the map in the browser.
Three.js displays the result.

1. Connect the D435 to Windows over USB 3. Mount the iPhone rigidly to the camera. Aim the
   iPhone's rear camera and D435 in the same direction. Viewed from the iPhone screen, put the
   phone's right edge up so its device +X points toward the top of the D435 image. Keep the
   devices close together. The existing axis conversion assumes this orientation and the
   estimator currently assumes zero offset between their centers.
2. From the fork root, install the dependencies. The main app needs the native image SLAM
   programs, which the standalone depth preview can run without.

   ```powershell
   npm ci
   git submodule update --init
   $env:SLAM_NATIVE_CACHE_ROOT = Join-Path $env:TEMP 'slam-web-phone/native'
   npm run assets
   py -3.12 -m pip install pyrealsense2 numpy websocket-client Pillow
   ```

3. Stop the old preview relay and sender if they are running. Start the updated relay, browser
   app and sender in three PowerShell windows from the fork root.

   ```powershell
   npm run live -- --host=0.0.0.0 --port=8788
   ```

   ```powershell
   npm run dev -- --host 127.0.0.1 --port 5174 --strictPort
   ```

   ```powershell
   py -3.12 live/realsense-preview.py --slam
   ```

4. Open [SLAM Lab](http://127.0.0.1:5174/?source=d435&live=ws://127.0.0.1:8788/runner) on
   the PC. In **Live sensors**
   choose **D435 color and depth + iPhone IMU**, pick **640 × 480** and click **Start live capture**.
5. On the same Wi-Fi, open the installed [SlamCapture app](../../ios/README.md) and enter the PC's
   LAN IP followed by `:8788`. Start streaming and keep the rig still for initialization.
   Use a signed iOS build. A build of the updated app skips sending iPhone images in this mode.
   With the original app the relay discards its image packets and accepts its IMU.

The camera sender timestamps each frameset when Windows receives it. The relay estimates a
fixed phone-to-PC clock offset from the first 12 IMU batches and waits for a covering IMU sample
before forwarding each frame. USB and Wi-Fi delay still affect alignment. A measured
correction can be passed to the sender as `--time-shift-ms`. The relay and browser must be
restarted for a clean test after swapping devices. Calibrate the relative rotation, translation
and time offset before relying on map accuracy, and move slowly during the first test.

Use `--record d435-rgbd.bag` on the sender to save the SDK color and Z16 depth streams before
alignment and JPEG encoding. The browser input uses the existing Core Motion device-motion
stream. It contains processed acceleration and rotation rate. Raw independent IMU samples
require a separate capture path.
