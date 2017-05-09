# LBridge
LBridge - Read the Freestyle Libre sensor and send the BG readings to xDrip+ using the xBridge2 protocol

This project is is a modification/externsion of the LimiTTer projekt from JoernL. The code is running on the LimiTTer hardware platform which is described in the LimiTTer project.

LBridge uses the xBridge2 protocol from the Wixel project / implementation of savek-cc. The code improves the original LimiTTer in terms of

  - battery usage is minimized which gives a longer runtime
  - in case of BLE connect problems missed BG readings are queued and resended once BLE connection is reestablished
  - it contains some workarounds to improve BLE connection setup

LBridge is working with the great App xDrip+. You can find it here:

https://jamorham.github.io/#xdrip-plus

Click on "Download latest APK" or choose the "Nightly Snapshots" for the very latest ones.

Please note, that LimiTTer is NOT maintained by Abbott. It is a experimental DIY project. You will built your own individual LimiTTer. So you are responsible yourself for what you have built. I have opened this project here because it's a invitation for all developers out there to improve the LimiTTer.
