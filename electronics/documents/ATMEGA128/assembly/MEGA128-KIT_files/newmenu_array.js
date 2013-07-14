timegap=500
followspeed=5
followrate=40
suboffset_top=0;
suboffset_left=10;

effect = "fade(duration=0.3);Shadow(color='#777777', Direction=135, Strength=5)"

function openwin(url)
{
	nwin=window.open(url, "nwin",config="scrollbars=yes,resizable=yes,toolbar=yes,location=yes,status=yes,menubar=yes,");
	nwin.focus();
}

prop1=[						// prop1 is an array of properties you can have as many property arrays as you need
"ffffff",					// Off Font Color
"006699",					// Off Back Color
"ffffff",					// On Font Color
"443F5C",					// On Back Color
"999999",					// Border Color
10,							// Font Size
"bold",					// Font Style 
"bold",					// Font Weight
"Verdana,Tahoma,Helvetica",	// Font
4,							// Padding
"../../../menu/arrow.gif",				// Sub Menu Image
,							// 3D Border & Separator
"66ffff",					// 3D High Color
"000099",					// 3D Low Color
"000099",					// Referer item Font Color (leave this blank to disable)
"FFFF00",					// Referer item Back Color (leave this blank to disable)
]

menu1=[				// This is the array that contains your menu properties and details
25,					// Top
20,					// left
97,					// Width
0,					// Border Width
"left",			// Screen Position - here you can use "center;middle;right"
prop1,				// Properties Array - this is set higher up, as above
1,					// Always Visible - allows the menu item to be visible at all time
"center",			// Alignment - sets the menu elements alignment, HTML values are valid here for example: left, right or center
,					// Filter - Text variable for setting transitional effects on menu activation
,					// Follow Scrolling - Tells the menu item to follow the user down the screen
1, 					// Horizontal Menu - Tells the menu to be horizontal instead of top to bottom style
,					// Keep Alive - Keeps the menu visible until the user moves over another menu or clicks elsewhere on the page
,					// Position of sub image left:center:right:middle:top:bottom
,					// Show an image on top menu bars indicating a sub menu exists below
,					// Reserved for future use
"�������","../../../index.php",,,1,// "Description Text", "URL", "Alternate URL", "Status", "Separator Bar"
"� ��������","show-menu2",,"#",1,
"����-�������","../../../showroom.php",,,1,
"�������: �-�","../../../catalog.php",,,1,
"����� �����","../../../nav.php",,,1,
"����� �� �����","../../../poisk.php",,,1,
"��� ������","../../../order.php",,,1

]

menu2=[
,,170,1,"",prop1,,"left",effect,,,,,,,
"� ���","../../../right.php",,,1,
"�������� �����","../../../order.php",,,1,
"����� �������","../../../postavki/postavki.php",,,1,
"��������","../../../kontakts.php",,,1
]

	menu4=[
	,,120,1,"",prop1,,"left",effect,,,,,,,
	"CTV News","http://www.ctvnews.com",,,1,
	"Vancouver Sun","http://www.canada.com/vancouver/vancouversun/",,,1
	]

menu3=[
,,170,1,"",prop1,,"left",effect,,,,,,,
"������������","show-menu5",,,1,
"�������� ����������","show-menu6",,,1,
"������������ ����������","show-menu19",,,1,
"��������� ����������","show-menu17",,,1,
"�������","show-menu7",,,1,
"����������������","show-menu8",,,1,
"�������� ������������","show-menu9",,,1,
"������ ����������� ������, �����","show-menu10",,,1,
"��������� �������","show-menu11",,,1,
"����������� ����������","show-menu13",,,1,
"����","show-menu12",,,1,
"����������� ����������","show-menu14",,,1,
"��� ��� �������","show-menu15",,,1,
"������������� �������","show-menu18",,,1,
"�������������� �������, �������","show-menu20",,,1,
"������ ������-���","../../../masterkit/home-p1.php",,,1
]

menu5=[
,,350,1,"",prop1,,"left",effect,,,,,,,
"��� �������� ����������","../../../razrabotka/menu.php",,,1,
"��������� ����� STK500","../../../data/stk500.php",,,1,
"������������ MSP-FET430P410","../../../data/FET430P410.php",,,1,
"3 � 1 - ������������� ������������ ATMEL AVR ISP/ +5V �������� �������","../../../razrabotka/avr-prog-isp.php",,,1,
"4 � 1 - ������������� ������������ ATMEL AVR ISP v3.2/ ������������� ������������� +5V �������� �������/ ��������������� USB � UART_TTL (5� ������)/�������� �������� ������� 1,8���","../../../razrabotka/avr_isp_3_2.php",,,1,
"EXTRA-PIC v2.0 - ������������ PIC ������������ � I2C EEPROM","../../../razrabotka/extra-pic.php",,,1,
"USB/RS232 AVR JTAG ICE-������������� �������� � ������������","../../../razrabotka/avr-jtag-usb-rs232.php",,,1,
"����������� ���������� �������� ��� MSP430","../../../news/EZ430-F2013.php",,,1,
"���������� ����� AT91SAM7S128","../../../razrabotka/91sam7s128_db_lite.php",,,1,
"PIC-������������","../../../data/pic-prog.php",,,1,
"AVR-TINY-MEGA-������������","../../../data/avr-prog.php",,,1,
"ARM-JTAG ������������/��������","../../../razrabotka/ARM-JTAG-w.php",,,1,
"�������� AVR JTAG","../../../data/jtag.php",,,1,
"�������� AVR-Easy","../../../data/avr-easy.php",,,1,
"�������� PIC-Easy","../../../data/pic-easy.php",,,1,
"���������� �������� Jennic JN5121-EK000","../../../razrabotka/jn5121.php",,,1,
"�������������� ��� ����������� ������ �� ��� �� TEXAS INSTRUMENTS - TMDSDSK5509-0E ","../../../razrabotka/tmdsdsk5509-0e.php",,,1,
"�������� ���������� � ������� ��� ����������� FTDI","../../../data/ftdi/ftdi.php",,,1,
"USBLCD 2002 - �������� ��� ������ ��","../../../data/usblcd.php",,,1,
"�������� �����","../../../razrabotka/maket/avr-p20.php",,,1
]

	menu6=[
	,,290,1,"",prop1,,"left",effect,,,,,,,
	"����� � ������","../../../data/fastdiod.php",,,1,
	"��������������� ����� (TRANSIL)","../../../data/smbj.php",,,1,
	"���������� ������","../../../data/memory/dram_mikron.php",,,1,
	"RFID - �������������� �������������","../../../data/rfid.php",,,1,
	"RFID - RR08U - USB-�����������","../../../data/rr08u.php",,,1,
	"IGBT-������ Semikron","../../../data/semikron/semitrans/semitrans.php",,,1,
	"IGBT-������ International Rectifier","../../../data/igbtmod/igbt.php",,,1,
	"IRAMxx - ���������������� IGBT-������","../../../data/iram.php",,,1,
	"HEXFET-������� ���-�����������","../../../data/irf/n_channel.php",,,1,
	"��������� �������� ����������","../../../data/ion.php",,,1,
	"��������������, �������","../../../trans.php",,,1,
	"�������������� ��� ��������� �������","../../../data/bv/bv.php",,,1,
	"���-�����������","../../../data/pi.php",,,1,
	"���������, ���������","../../../data/simistor.php",,,1,
	"����������� IGBT","../../../data/igbt.php",,,1,
	"����� ������","../../../data/shottki.php",,,1,
	"������������ ��������� MAXIM","../../../data/oumax.php",,,1,
	"�������� ������� �������","../../../data/timer.php",,,1,
	"�������� �������� SHARP","../../../data/sharp/sttr.php",,,1,
	"�������������","../../../stab/stab.php",,,1,
	"������������ �����, �����������","../../../data/irbrt.php",,,1,
	"��������� �������","../../../table.php?name=solarmodul&page=0",,,1,
	"���������� �������� ���������","../../../data/cncstepdriver/cnc.php",,,1,
	"��������� ����������","../../../data/geyer/geyer.php",,,1,	
	"��������� ����������","../../../data/geyer/generator.php",,,1,	
	"������������, �������","../../../data/eemb/li_bat.php",,,1,	
	"��������������� ���������� �������� Sensata","../../../data/sensata.php",,,1,
	"��������� WizNet - ������ NM7010A-LF","../../../data/NM7010A-LF.php",,,1,
	"�������� �������","../../../laser/laser.php",,,1
	]

	menu7=[
	,,220,1,"",prop1,,"left",effect,,,,,,,
	"������� ���������","../../../data/vlag.php",,,1,
	"������� ��������","../../../data/davl.php",,,1,
	"������� ����������� HoneyWell","../../../data/termo-h.php",,,1,
	"������� ����������� HoneyWell - ����������","../../../data/hel.php",,,1,
	"������� �������� HoneyWell","../../../data/honeydavl.php",,,1,
	"������� ������� ���� HoneyWell","../../../data/rashod/rashod.php",,,1,
	"������� ������������� � �����������","../../../data/qtouch/qt100.php",,,1,
	"������� ������������, �����","../../../data/apds/apds.php",,,1,
	"������������ ������� ��������� � �������� �������� HoneyWell","../../../data/hoa/hoasens.php",,,1,
	"������ ���� ��������","../../../data/HRS100SSAB090.php",,,1,
	"������� (���������) �����","../../../data/gas/gas.php",,,1,
	"������� ����������� Dallas","../../../termo_dallas.php",,,1,
	"������� ����������� PHILIPS","../../../data/kty.php",,,1,
	"������� ����������� National","../../../data/nscterm.php",,,1,
	"������� ������ ��������","../../../data/lle.php",,,1,
	"������� ���������","../../../data/uskorenie.php",,,1,
	"������� �����","../../../data/hall.php",,,1,
	"������� �������","../../../data/d7e3.php",,,1,
	"������������� ���� - AS-405","../../../data/as-405.php",,,1,
	"������� ���� Allegro Microsystems","../../../data/allegropower.php",,,1,
	"������� ���� HoneyWell","../../../data/tok.php",,,1
	]

	menu8=[
	,,300,1,"",prop1,,"left",effect,,,,,,,
	"MSP430-����� �� Texas Instruments","../../../data/msp430.php",,,1,
	"���������������� MSP430F2xx","../../../data/msp430f2xx.php",,,1,
	"������������ ���������� DaVinci","../../../data/davinci.php",,,1,
	"8051 �������� Silicon Labs","../../../data/slabs.php",,,1,
	"Parallax: 8-�� ������� ����������� P8X32a","../../../data/p8x32a.php",,,1,
	"���������������� ATMEL","../../../avr/about.php",,,1,
	"���������������� ATMEL ����� AT91","../../../data/arm/arm.php",,,1,
	"����������� ���������������� MICROCHIP ����� PIC10","../../../data/pic10.php",,,1,
	"���������������� PHILIPS ����� LPC2000","../../../data/lpc/lpc2000.php",,,1
	]

	menu9=[
	,,300,1,"",prop1,,"left",effect,,,,,,,
	"���! �������� ������� LUKEY-702","../../../data/lukey-702.php",,,1,
	"���! �������� ������� LUKEY-852D+FAN","../../../data/lukey-852dplus.php",,,1,
	"����� ��������� �������","../../../data/trubprip.php",,,1,
	"������, �����, �������� �����","../../../data/pripoj.php",,,1,
	"�������� �������","../../../payal/LUKEY-852D.php",,,1,
	"�������� �������-����������","../../../payal/stdop.php",,,1,
	"��������� � ������������","../../../payal/payal/8PK-S110B-40W.php",,,1,
	"�����������","../../../nakon/nak.php",,,1,
	"������������� ��������������","../../../data/wick.php",,,1,
	"�������� ����� - ��� ������","../../../plata.php",,,1
	]

	menu10=[
	,,170,1,"",prop1,,"left",effect,,,,,,,
	"����� ��� �����������","../../../him/him.php",,,1,
	"���������","../../../data/fnr.php",,,1,
	"C���������������������� �������������� ROHM","../../../data/icp.php",,,1,
	"�������������� polyswitch","../../../ruerxe.php",,,1,
	"�������������� �������","../../../table.php?name=1170203&page=0",,,1,
	"����������������������� �������������� ������������ BOURNS","../../../data/mf-r.php",,,1,
	"�������������������","../../../termo.php",,,1
	]

	menu11=[
	,,170,1,"",prop1,,"left",effect,,,,,,,
	"������������ ��������� �������","../../../power/power.php",,,1,
	"��������","../../../power/about.php",,,1,
	"Recom","../../../power/dcdc-recom.php",,,1,
	"Chinfa","../../../power/dcdc-chinfa.php",,,1,
	"Mean Well","../../../power/acdc-meanwell.php",,,1,
	"���������� ��������� ���������� ����������","../../../power/rs-power.php",,,1,
	"��������� Mean Well","../../../power/inverter/inverter.php",,,1,
	"������� �������� 18-25 �� Mean Well","../../../power/es18.php",,,1,
	"�������� ����������","../../../power/charger.php",,,1,
	"Motien","../../../power/motien.php",,,1,
	"DC / DC ��������������� PEAK electronics GmbH","../../../power/peak.php",,,1,
	"���������� ��������","../../../power/acdc-ae.php",,,1
	]

	menu12=[
	,,170,1,"",prop1,,"left",effect,,,,,,,
	"���� ����������","",,,1,
	"���� �������������������","../../../relay/wanjia.php",,,1,
	"������������� ���� COSMO","../../../data/ksa.php",,,1,
	"������������� ���� International Rectifier","../../../data/ttr.php",,,1,
	"������������� ���� SHARP","../../../data/sharp/sttr.php",,,1
	]

	menu13=[
	,,270,1,"",prop1,,"left",effect,,,,,,,
	"������������ ����������","../../../data/led/1dig.php",,,1,
	"����������","../../../data/light/leds.php?name=leds&page=0",,,1,
	"������������ ����������","../../../product.php?name=infrared&page=0",,,1,
	"�����-��������������� ���������� - ������ ��������","../../../table.php?name=104080302&page=0",,,1,
	"���������� ��� �������������� �������","../../../data/rohmled/smdled.php",,,1,
	"���������� �� ������","../../../gki/intech/intech.php",,,1,
	"PLED (OLED)-����������","../../../gki/pled.php",,,1,
	"������������ ���","../../../gki/bistab.php",,,1
	]

	menu14=[
	,,170,1,"",prop1,,"left",effect,,,,,,,
	"��� ��� ������� � ������������","../../../rs/rem.php",,,1,
	"������������ �� CD-ROM","../../../support.php",,,1
	]

	menu15=[
	,,170,1,"",prop1,,"left",effect,,,,,,,
	"�������� �������","../../../lh/lh.php",,,1,
	"��������� ��������� ��� LCD �������","../../../data/invertled.php",,,1,
	"�������� ��� ���-�����","",,,1
	]

	menu16=[
	,,170,1,"",prop1,,"left",effect,,,,,,,
	"�������� �����","../../../guest/guestbook.php",,,1
	]

	menu17=[
	,,170,1,"",prop1,,"left",effect,,,,,,,
	"��������� ������������","../../../data/vatronics.php",,,1,
	"���������� ������������","../../../data/tantal.php",,,1,
	"EPCOS - ���������� ����������","../../../table.php?name=ferrite&page=0",,,1,
	"������� �������������� 15 � 18 ����","../../../table.php?name=transpower&page=0",,,1,
	"����������������� ������������ Hitachi AIC","../../../data/hit-aic/hit-aic.php",,,1,
	"������� �������������","../../../data/sumida/sumida.php",,,1,
	"����������","../../../data/keyboard/key.php",,,1,
	"�����������","../../../data/fun/fun.php",,,1

	]

	menu18=[
	,,270,1,"",prop1,,"left",effect,,,,,,,
	"����������� ����������� 10 ��� - HPS-10","../../../data/pribor/hps-10.php",,,1,
	"����������� ����������� 40 ��� - HPS-40","../../../data/pribor/hps-40.php",,,1,
	"USB ����������� PV6501","../../../data/pv6501.php",,,1,
	"������������� �������","../../../table.php?name=pribor&page=0",,,1,	
	"���� ��� ������������","../../../data/hp.php",,,1,
	"���� ��� ������������� ��������","../../../table.php?name=10510&page=0",,,1,
	"���� �����������","../../../data/ps-x00.php",,,1,
	"���������� ��������","../../../data/ivpa.php",,,1,
	"�������� ��������� ETP-104","../../../data/s-line/etp-104.php",,,1,
	"�������������� ��300","../../../data/tp300.php",,,1,
	"�������� ����������","../../../data/nivelir/nivelir.php",,,1
	]

	menu19=[
	,,250,1,"",prop1,,"left",effect,,,,,,,
	"�������! GSM-������","../../../data/fsu.php",,,1,
	"GSM-W1 �������� GSM-�������","../../../data/gsm-w1.php",,,1,
	"������������ �����������","../../../data/zigbee/zigbeemodem.php",,,1,
	"����������� ��� ������ ����������","../../../data/saturn.php",,,1,
	"GSM/UMTS/GPS - ����������","../../../data/gsmaccessories/antgsm.php",,,1,
	"G P S - ���������","../../../data/gps/gps.php",,,1,
	"BlueTooth ������","../../../data/bluetooth/bluetooth.php",,,1,
	"������������ BlueTooth ������ �������� SENA","../../../data/bluetooth/sena.php",,,1,
	"����������� (���������) ��������� 433 ��� � ��������� ����������","../../../data/telecontroli/telecontroli.php",,,1,
	"ZigBee-����������","../../../data/chipcon/cc1000.php",,,1,
	"�������� �� ������������ �����������","../../../data/gsmmodem/gsmcatalog.php",,,1
	]

	menu20=[
	,,250,1,"",prop1,,"left",effect,,,,,,,
	"������� DIN 41612","../../../data/41612/?name=612b",,,1,
	"������������ ��������","../../../data/klemnik/klem.php",,,1,
	"��������� ���������������","../../../table.php?name=1040112&page=0",,,1,
	"��������� ���� ������","../../../data/memorycard/CCM05-5501LFT-R901.php",,,1,
	"������������ ������","../../../data/ffc.php",,,1,
	"GAINTA -  ������������������ ������� ��� ���","../../../table.php?name=1040602&page=0",,,1,
	"���������� ������ � ��������� �������������","../../../data/bathold.php",,,1
	]

