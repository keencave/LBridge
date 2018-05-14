# LBridge
LBridge - Read the Freestyle Libre sensor and send the BG readings to xDrip+ using the xBridge2 protocol

This project is is a modification/extension of the LimiTTer projekt from @JoernL. The code is running on the LimiTTer hardware platform which is described in the LimiTTer project. It replaces the LimiTTer code and provide the same functionalities.

LBridge uses the xBridge2 protocol from the Wixel project / implementation of savek-cc. It improves the original LimiTTer in terms of

  - optimized battery usage
  - no missed BG readings
  - better BLE performance

If you have questions please join the gitter channel: https://gitter.im/JoernL/LimiTTer

LBridge is working with the great App xDrip+. You can find it here:

https://jamorham.github.io/#xdrip-plus

Click on "Download latest APK" or choose the "Nightly Snapshots" for the very latest ones.

Please note, that LBridge code for LimiTTer is NOT maintained by Abbott. It is an experimental DIY project. You will built your own individual LimiTTer. So you are responsible yourself for what you have built. This is not a medical device. Dont make any medical decisions based on the results as they can be wrong!

Files:

RFduino/Simblee platforms (different DIY PCBs from Marek Macner, chaosbiber, Thomacz STachowicz available)

LBridge_RFduino_V0.9.19_180426_1554:
  - code freeze of V0.9 version, default seetings for instant use with xAPS
  - full 8h backfilling (get missed readings from the Libre Sensor like LibreAlarm after missing sensor)
  - shadow FRAM mechanism from @UPetersen
  - switch off serial interface due ULP sleep phase for better power consumption, @Chaosbiber
  - code aligned to Mareks xbridgeM code spin off for his T-Mini project
  - better BLE performance, applied some work arounds
  - improved lifetime (up to 5 days with 240 mAh battery)

Original LimiTTer Arduino Pro Mini platform (no modifications needed)

LBridge_Arduino_170716_1950.ino: release candidate, please use the latest Arduino IDE to compile
  - improved BLE connectivity. This can cost more battery power under bad BLE conditions
  - default name set to "LimiTTer" to avoid hardware source changing to xbridge wixel in xDrip+ 
  - added detection of sensor lifetime. Automatic stop after 20880 operation (=14,5 days) or 5 identical glucose readings
  - added spike filter (set to +- 40 mg/dL)
  - added sensor lifetime display in xDrip+ (works on phone, not on SSW3)
  - NFC read code modified to avoid possible sensordeaths due to massive re-read tries of BM019
  - autocalibrate PWR DWN timer to get accurate 5 min spacing
  - increased queue depth for BG readings to 5h
  - bugfix for detecting dead sensors before end of lifetime
  
