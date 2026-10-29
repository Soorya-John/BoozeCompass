# Windows ESP-IDF setup

## Environment inspected for this checkout

- Windows 10 Home Single Language, build reported as 26220.
- Windows PowerShell 5.1.
- Desktop resolved by Windows as C:\Users\soory\Desktop; it is not the OneDrive desktop.
- ESP-IDF v5.2.8 at C:\Espressif\frameworks\esp-idf-v5.2.8.
- ESP-IDF Python environment: Python 3.11.2.
- C3 compiler: RISC-V GCC 13.2.0.
- Git for Windows and GitHub CLI are installed.

## Activate the matching IDF

Use the installed ESP-IDF 5.2 PowerShell Desktop shortcut, or run:

    powershell.exe -ExecutionPolicy Bypass -NoProfile
    . C:\Espressif\Initialize-Idf.ps1 -IdfId esp-idf-6dce30f5bfd7d7db2f0573e74d784ece

That IDF identifier belongs to this machine's Espressif installation; another machine should use its own ESP-IDF PowerShell shortcut/identifier. Once activated:

    Set-Location "$env:USERPROFILE\Desktop\BoozeCompass"
    idf.py --version
    idf.py set-target esp32c3
    idf.py menuconfig
    idf.py build
    idf.py -p COM5 flash
    idf.py -p COM5 monitor

The local execution policy can block scripts in ordinary PowerShell. The installed shortcut uses -ExecutionPolicy Bypass; it does not change the system policy.

## Configuration

Set SSID and password under BoozeCompass → Network and OpenStreetMap in menuconfig. sdkconfig is generated locally, ignored by Git, and may contain credentials in plaintext; do not commit or share it. Configure GPIO pins only after verifying the board revision and wiring notes.

To return to a clean ESP32-C3 configuration if the target changes:

    idf.py fullclean
    idf.py set-target esp32c3
    idf.py menuconfig
    idf.py build

For serial output, use the port shown by Device Manager. ESP-IDF's monitor exits with Ctrl+].
