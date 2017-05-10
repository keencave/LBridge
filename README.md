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

Lbridgev_PR_170302.ino, Lbridgev_170316: These files were provided in the LimiTTer gitter discussion group in march. Please refer to the chat protocols to get details on them.

I used the 0316 version the last 2 months without problems in my live system.

Lbridge_R_170510: Contains some improved handling for the HM-11 module in terms of timing/handling to ensure better BLE performance. I started testing.
