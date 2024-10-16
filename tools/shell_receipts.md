# Configure LEDs (done by driver)
```
gpio conf pcf8574t@20 4 ol0
gpio conf pcf8574t@20 5 ol0
gpio conf pcf8574t@20 6 ol0
```
# Turn on/off  POW LED
```
gpio set pcf8574t@20 4 1
gpio set pcf8574t@20 4 0
```
# Configure relays (done by driver)
```
gpio conf pcf8574t@20 0 oh0
gpio conf pcf8574t@20 1 oh0
gpio conf pcf8574t@20 2 oh0
gpio conf pcf8574t@20 3 oh0
```
# Turn on/off relay 1
```
gpio set pcf8574t@20 0 1
gpio set pcf8574t@20 0 0
```
# Get relays output state
```
gpio get pcf8574t@20 0
gpio get pcf8574t@20 1
gpio get pcf8574t@20 2
gpio get pcf8574t@20 3
```
# turn off/on LAN8720 power
```
gpio set gpio@3ff44000 0 0
gpio set gpio@3ff44000 0 1
```
# read modbus slave
```
modbus read inpregs 10 50 4
```
# read external rtc
```
rtc get rv3032@51
```
# mount FAT on SD/SDHC/MMC (done by app)
```
fs mount fat /SD:
```
# test FAT on SD/SDHC/MMC
```
fs erase_write_test /SD:/wrtest.txt 50000 5
fs read_test /SD:/wrtest.txt 5
fs ls /SD:
fs statvfs /SD:
```
# test LittleFS on internal flash 'storage_partition'
```
fs write /LFS:/wrtest.txt 11 22 33 44 55 66 77 88 99 00 aa
fs read /LFS:/wrtest.txt
fs read_test /LFS:/wrtest.txt 5
fs ls /LFS:
fs statvfs /LFS:
```
# other
```
kernel reboot warm
devmem dump -a 3f400000 -s 128
```