wch-isp(1) "VERSION"

# NAME

wch-isp - firmware programmer for WCH microcontrollers

# SYNOPSIS

*wch-isp* [_OPTIONS_] _COMMAND_ [_ARG_...]

*wch-isp* [*-VDnpr*] [*-d* _UID_] [*flash*|*write*|*verify*|*reset*] _FILE_

*wch-isp* [*-VDnpr*] [*-d* _UID_] [*erase*|*config*|*remove-wp*]

*wch-isp* [*-VDnpr*] [*-d* _UID_] *config-set* _REG_NAME_[*:*_FIELD_SEL_]*=*_VALUE_

*wch-isp* [*-VDnpr*] [*-d* _UID_] [*list*]

# DESCRIPTION

*wch-isp* is an utility to write firmware into the flash of WCH
microcontrollers, over USB.  By default the flash content is verified
after writing.

# OPTIONS

*-d* _UID_
	Select the USB device that matches the _UID_. If nothing matches
	then _UID_ is used as an index to select a device from the list.

*-n*
	Do not verify flash content after writing, verification is done
	by default.

*-p*
	Prints a progress bar during command operation.

*-r*
	Reset the microcontroller after the command is completed.

*-D*
	Enable debug information, print raw isp commands sent over USB.

*-V*
	Prints version information to stdout, then exits.

# COMMANDS

*list*
	List the currently detected compatible USB devices.

*write* _FILE_
	Write _FILE_ into flash, this will automatically erase sufficient
	flash sectors before writing to it.

*flash* _FILE_
	Write _FILE_ into flash, same as write.

*verify* _FILE_
	Verify _FILE_ content against the flash content.

*reset*
	Reset the microcontroller and jump to code flash.

*erase*
	Erase all the flash content.

*config*
	Print a memory dump of the chip configuration. When supported the config
	is decoded to show registers name with it's fields value.

*config-set* _REG_NAME_[*:*_FIELD_SEL_]*=*_VALUE_
	Change the value of a config field. _REG_NAME_ designate the name of
	the register to modify in the config (see *config* output).
	_FIELD_SEL_ is optional and can either be a number *[*_[0-9]+_*]*
	a range *[*_[0-9]+:[0-9]+_*]* or the name of a field.

*remove-wp*
	Disable flash write protection (modifies the chip configuration).

# EXAMPLES

List detected devices:
```
$ wch-isp list
0: BTVER v2.7 UID 8d-ff-ba-e4-c2-84-09-69 [0x1069] CH569
1: BTVER v2.5 UID f2-3e-88-26-3b-38-b5-9d [0x1980] CH32V208WBU6
```

Flash the *firmware.bin* file, *-p* enables the progress bar:
```
$ wch-isp -p flash firmware.bin
BTVER v2.5 UID f2-3e-88-26-3b-38-b5-9d [0x1980] CH32V208WBU6
[####################################################] write 35392/35392
[####################################################] verify 35392/35392
flash done
```

Erase the device's flash, select the device by it's UID (option *-d*):
```
$ wch-isp -d f2-3e-88-26-3b-38-b5-9d erase
BTVER v2.5 UID f2-3e-88-26-3b-38-b5-9d [0x1980] CH32V208WBU6
erase done
```

Change the config to disable the read protection *RDPR* and change the
*STOP_RST* bit:
```
$ wch-isp config-set RDPR=0xa5 USER:STOP_RST=0
BTVER v2.6 UID cd-ab-bb-76-04-bc-8c-de [0x153f] CH32V103x6x6 (flash 32K)
writing new config
config-set done
$ wch-isp config
BTVER v2.6 UID cd-ab-bb-76-04-bc-8c-de [0x153f] CH32V103x6x6 (flash 32K)
raw: a5 5a ff 00 ff ff ff ff ff ff ff ff
00: RDPR: 0xa5
    [7:0]: 0xa5 (disabled)
02: USER: 0xff
      [5] PORCTR: 1 (0b1)
      [2] STANDBY_RST: 1 (disabled)
      [1] STOP_RST: 0 (enabled)
      [0] IWDG_SW: 1 (disabled)
04: DATA0: 0xff
06: DATA1: 0xff
08: WRPR0: 0xff
09: WRPR1: 0xff
0a: WRPR2: 0xff
0b: WRPR3: 0xff
config done
```
