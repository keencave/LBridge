# LBridge
LBridge - Read the Freestyle Libre sensor and send the BG readings to xDrip+ using the xBridge2 protocol

This project is is a modification/extension of the LimiTTer projekt from JoernL. The code is running on the LimiTTer hardware platform which is described in the LimiTTer project. It replaces the LimiTTer code and provide the same functionalities.

LBridge uses the xBridge2 protocol from the Wixel project / implementation of savek-cc. It improves the original LimiTTer in terms of

  - optimized battery usage
  - no missed BG readings
  - better BLE performance

LBridge is working with the great App xDrip+. You can find it here:

https://jamorham.github.io/#xdrip-plus

Click on "Download latest APK" or choose the "Nightly Snapshots" for the very latest ones.

Please note, that LBridge code for LimiTTer is NOT maintained by Abbott. It is an experimental DIY project. You will built your own individual LimiTTer. So you are responsible yourself for what you have built. This is not a medical device. Dont make any medical decisions based on the results as they can be wrong!

Files:

Lbridgev_PR_170302.ino: This version was provided in the LimiTTer gitter discussion group in march. Please refer to the chat protocols to get details on them.

LBridgev_PR_170311.ino: Improved HM-11 handling. I used this version the last 2 months without problems in my live system. Stable and reliable. Published in gitter discussion group as version 0316. Please refer to the chat protocols to get more details.

Lbridge_R_170510_3.ino: Tested with HM-11 (V547) and HM-17 (V117). Improved timings to reduce battery consumption. Fast handover between Wear Collection Service and xDrip+.

LBridge_Arduino_170627_0011.ino: Improved BLE connectivity. This can cost more battery power under bad BLE conditions. Removed LimiTTer data smoothing. Default name set to LimiTTer to avoid xDrip+ problems. Autocalibrate PWR DWN timer to get accurate 5 min spacing. Please use the latest Arduino IDE to compile this INO. 
