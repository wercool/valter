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
"Главная","../../../index.php",,,1,// "Description Text", "URL", "Alternate URL", "Status", "Separator Bar"
"О компании","show-menu2",,"#",1,
"Фото-Каталог","../../../showroom.php",,,1,
"Каталог: А-Я","../../../catalog.php",,,1,
"Карта сайта","../../../nav.php",,,1,
"Поиск по сайту","../../../poisk.php",,,1,
"Как купить","../../../order.php",,,1

]

menu2=[
,,170,1,"",prop1,,"left",effect,,,,,,,
"О нас","../../../right.php",,,1,
"Оформить заказ","../../../order.php",,,1,
"Линия поствок","../../../postavki/postavki.php",,,1,
"Контакты","../../../kontakts.php",,,1
]

	menu4=[
	,,120,1,"",prop1,,"left",effect,,,,,,,
	"CTV News","http://www.ctvnews.com",,,1,
	"Vancouver Sun","http://www.canada.com/vancouver/vancouversun/",,,1
	]

menu3=[
,,170,1,"",prop1,,"left",effect,,,,,,,
"Разработчику","show-menu5",,,1,
"Активные компоненты","show-menu6",,,1,
"Беспроводные технологии","show-menu19",,,1,
"Пассивные компоненты","show-menu17",,,1,
"Датчики","show-menu7",,,1,
"Микроконтроллеры","show-menu8",,,1,
"Паяльное оборудование","show-menu9",,,1,
"Защита электронных систем, химия","show-menu10",,,1,
"Источники питания","show-menu11",,,1,
"Отображение информации","show-menu13",,,1,
"Реле","show-menu12",,,1,
"Техническая литература","show-menu14",,,1,
"Все для ремонта","show-menu15",,,1,
"Измерительная техника","show-menu18",,,1,
"Коммутационные изделия, корпуса","show-menu20",,,1,
"Наборы МАСТЕР-КИТ","../../../masterkit/home-p1.php",,,1
]

menu5=[
,,350,1,"",prop1,,"left",effect,,,,,,,
"ВСЕ СРЕДСТВА РАЗРАБОТКИ","../../../razrabotka/menu.php",,,1,
"стартовый набор STK500","../../../data/stk500.php",,,1,
"программатор MSP-FET430P410","../../../data/FET430P410.php",,,1,
"3 в 1 - Внутрисхемный программатор ATMEL AVR ISP/ +5V источник питания","../../../razrabotka/avr-prog-isp.php",,,1,
"4 в 1 - Внутрисхемный программатор ATMEL AVR ISP v3.2/ гальванически изолированный +5V источник питания/ преобразователь USB в UART_TTL (5В уровни)/Источник тактовой частоты 1,8МГц","../../../razrabotka/avr_isp_3_2.php",,,1,
"EXTRA-PIC v2.0 - Программатор PIC контроллеров и I2C EEPROM","../../../razrabotka/extra-pic.php",,,1,
"USB/RS232 AVR JTAG ICE-внутрисхемный отладчик и программатор","../../../razrabotka/avr-jtag-usb-rs232.php",,,1,
"Миниатюрное отладочное средство для MSP430","../../../news/EZ430-F2013.php",,,1,
"Отладочная плата AT91SAM7S128","../../../razrabotka/91sam7s128_db_lite.php",,,1,
"PIC-программатор","../../../data/pic-prog.php",,,1,
"AVR-TINY-MEGA-программатор","../../../data/avr-prog.php",,,1,
"ARM-JTAG программатор/отладчик","../../../razrabotka/ARM-JTAG-w.php",,,1,
"отладчик AVR JTAG","../../../data/jtag.php",,,1,
"отладчик AVR-Easy","../../../data/avr-easy.php",,,1,
"отладчик PIC-Easy","../../../data/pic-easy.php",,,1,
"Отладочный комплекс Jennic JN5121-EK000","../../../razrabotka/jn5121.php",,,1,
"Инструментарий для экономичных систем на ЦСП от TEXAS INSTRUMENTS - TMDSDSK5509-0E ","../../../razrabotka/tmdsdsk5509-0e.php",,,1,
"Средства разработки и отладки для компонентов FTDI","../../../data/ftdi/ftdi.php",,,1,
"USBLCD 2002 - примочка для Вашего ПК","../../../data/usblcd.php",,,1,
"макетные платы","../../../razrabotka/maket/avr-p20.php",,,1
]

	menu6=[
	,,290,1,"",prop1,,"left",effect,,,,,,,
	"Диоды и сборки","../../../data/fastdiod.php",,,1,
	"Ограничительные Диоды (TRANSIL)","../../../data/smbj.php",,,1,
	"Микросхемы памяти","../../../data/memory/dram_mikron.php",,,1,
	"RFID - Радиочастотная идентификация","../../../data/rfid.php",,,1,
	"RFID - RR08U - USB-считыватель","../../../data/rr08u.php",,,1,
	"IGBT-модули Semikron","../../../data/semikron/semitrans/semitrans.php",,,1,
	"IGBT-модули International Rectifier","../../../data/igbtmod/igbt.php",,,1,
	"IRAMxx - интеллектуальные IGBT-модули","../../../data/iram.php",,,1,
	"HEXFET-Силовые МОП-транзисторы","../../../data/irf/n_channel.php",,,1,
	"Источники опорного напряжения","../../../data/ion.php",,,1,
	"Трансформаторы, фильтры","../../../trans.php",,,1,
	"Трансформаторы для печатного монтажа","../../../data/bv/bv.php",,,1,
	"ШИМ-контроллеры","../../../data/pi.php",,,1,
	"Тиристоры, симисторы","../../../data/simistor.php",,,1,
	"Транзисторы IGBT","../../../data/igbt.php",,,1,
	"Диоды Шоттки","../../../data/shottki.php",,,1,
	"Операционные усилители MAXIM","../../../data/oumax.php",,,1,
	"Средства отсчета времени","../../../data/timer.php",,,1,
	"Оптореле компании SHARP","../../../data/sharp/sttr.php",,,1,
	"Стабилизаторы","../../../stab/stab.php",,,1,
	"Инфракрасные диоды, транзисторы","../../../data/irbrt.php",,,1,
	"Солнечные батареи","../../../table.php?name=solarmodul&page=0",,,1,
	"Контроллер шагового двигателя","../../../data/cncstepdriver/cnc.php",,,1,
	"Кварцевые резонаторы","../../../data/geyer/geyer.php",,,1,	
	"Кварцевые генераторы","../../../data/geyer/generator.php",,,1,	
	"Аккумуляторы, батареи","../../../data/eemb/li_bat.php",,,1,	
	"Биметаллические термостаты компании Sensata","../../../data/sensata.php",,,1,
	"Продукция WizNet - Модуль NM7010A-LF","../../../data/NM7010A-LF.php",,,1,
	"Лазерная техника","../../../laser/laser.php",,,1
	]

	menu7=[
	,,220,1,"",prop1,,"left",effect,,,,,,,
	"Датчики влажности","../../../data/vlag.php",,,1,
	"Датчики давления","../../../data/davl.php",,,1,
	"Датчики температуры HoneyWell","../../../data/termo-h.php",,,1,
	"Датчики температуры HoneyWell - дополнение","../../../data/hel.php",,,1,
	"Датчики давления HoneyWell","../../../data/honeydavl.php",,,1,
	"Датчики расхода газа HoneyWell","../../../data/rashod/rashod.php",,,1,
	"Датчики прикосновения и приближения","../../../data/qtouch/qt100.php",,,1,
	"Датчики освещенности, цвета","../../../data/apds/apds.php",,,1,
	"Инфракрасные датчики положения и скорости вращения HoneyWell","../../../data/hoa/hoasens.php",,,1,
	"Датчик угла поворота","../../../data/HRS100SSAB090.php",,,1,
	"Датчики (детекторы) газов","../../../data/gas/gas.php",,,1,
	"Датчики температуры Dallas","../../../termo_dallas.php",,,1,
	"Датчики температуры PHILIPS","../../../data/kty.php",,,1,
	"Датчики температуры National","../../../data/nscterm.php",,,1,
	"Датчики уровня жидкости","../../../data/lle.php",,,1,
	"Датчики ускорения","../../../data/uskorenie.php",,,1,
	"Датчики холла","../../../data/hall.php",,,1,
	"Датчики наклона","../../../data/d7e3.php",,,1,
	"Трансформатор тока - AS-405","../../../data/as-405.php",,,1,
	"Датчики тока Allegro Microsystems","../../../data/allegropower.php",,,1,
	"Датчики тока HoneyWell","../../../data/tok.php",,,1
	]

	menu8=[
	,,300,1,"",prop1,,"left",effect,,,,,,,
	"MSP430-серии от Texas Instruments","../../../data/msp430.php",,,1,
	"Микроконтроллеры MSP430F2xx","../../../data/msp430f2xx.php",,,1,
	"Многоядерные процессоры DaVinci","../../../data/davinci.php",,,1,
	"8051 компании Silicon Labs","../../../data/slabs.php",,,1,
	"Parallax: 8-ми ядерные контроллеры P8X32a","../../../data/p8x32a.php",,,1,
	"Микроконтроллеры ATMEL","../../../avr/about.php",,,1,
	"Микроконтроллеры ATMEL серии AT91","../../../data/arm/arm.php",,,1,
	"Миниатюрные микроконтроллеры MICROCHIP серии PIC10","../../../data/pic10.php",,,1,
	"Микроконтроллеры PHILIPS серии LPC2000","../../../data/lpc/lpc2000.php",,,1
	]

	menu9=[
	,,300,1,"",prop1,,"left",effect,,,,,,,
	"ХИТ! Паяльная станция LUKEY-702","../../../data/lukey-702.php",,,1,
	"ХИТ! Паяльная станция LUKEY-852D+FAN","../../../data/lukey-852dplus.php",,,1,
	"Выбор трубчатых припоев","../../../data/trubprip.php",,,1,
	"Припои, флюсы, паяльные пасты","../../../data/pripoj.php",,,1,
	"Паяльные станции","../../../payal/LUKEY-852D.php",,,1,
	"Паяльные станции-дополнение","../../../payal/stdop.php",,,1,
	"Паяльники и оборудование","../../../payal/payal/8PK-S110B-40W.php",,,1,
	"Наконечники","../../../nakon/nak.php",,,1,
	"Сопутствующие принадлежности","../../../data/wick.php",,,1,
	"Печатные платы - это просто","../../../plata.php",,,1
	]

	menu10=[
	,,170,1,"",prop1,,"left",effect,,,,,,,
	"Химия для электроники","../../../him/him.php",,,1,
	"Варисторы","../../../data/fnr.php",,,1,
	"Cамовосстанавливающиеся предохранители ROHM","../../../data/icp.php",,,1,
	"Предохранители polyswitch","../../../ruerxe.php",,,1,
	"Предохранители плавкие","../../../table.php?name=1170203&page=0",,,1,
	"Самовосстанавливающиеся предохранители производства BOURNS","../../../data/mf-r.php",,,1,
	"Термопредохранители","../../../termo.php",,,1
	]

	menu11=[
	,,170,1,"",prop1,,"left",effect,,,,,,,
	"ПРОМЫШЛЕННЫЕ ИСТОЧНИКИ ПИТАНИЯ","../../../power/power.php",,,1,
	"Введение","../../../power/about.php",,,1,
	"Recom","../../../power/dcdc-recom.php",,,1,
	"Chinfa","../../../power/dcdc-chinfa.php",,,1,
	"Mean Well","../../../power/acdc-meanwell.php",,,1,
	"Компактные источники повышенной надежности","../../../power/rs-power.php",,,1,
	"Инверторы Mean Well","../../../power/inverter/inverter.php",,,1,
	"Сетевые адаптеры 18-25 Вт Mean Well","../../../power/es18.php",,,1,
	"Зарядные устройства","../../../power/charger.php",,,1,
	"Motien","../../../power/motien.php",,,1,
	"DC / DC преобразователи PEAK electronics GmbH","../../../power/peak.php",,,1,
	"Александер Электрик","../../../power/acdc-ae.php",,,1
	]

	menu12=[
	,,170,1,"",prop1,,"left",effect,,,,,,,
	"Реле герконовые","",,,1,
	"Реле электромеханические","../../../relay/wanjia.php",,,1,
	"Твердотельные реле COSMO","../../../data/ksa.php",,,1,
	"Твердотельные реле International Rectifier","../../../data/ttr.php",,,1,
	"Твердотельные реле SHARP","../../../data/sharp/sttr.php",,,1
	]

	menu13=[
	,,270,1,"",prop1,,"left",effect,,,,,,,
	"Светодиодные индикаторы","../../../data/led/1dig.php",,,1,
	"СВЕТОДИОДЫ","../../../data/light/leds.php?name=leds&page=0",,,1,
	"Инфракрасные светодиоды","../../../product.php?name=infrared&page=0",,,1,
	"Жидко-кристаллические индикаторы - полный перечень","../../../table.php?name=104080302&page=0",,,1,
	"Светодиоды для поверхностного монтажа","../../../data/rohmled/smdled.php",,,1,
	"Сегментные ЖК панели","../../../gki/intech/intech.php",,,1,
	"PLED (OLED)-индикаторы","../../../gki/pled.php",,,1,
	"Бистабильные ЖКИ","../../../gki/bistab.php",,,1
	]

	menu14=[
	,,170,1,"",prop1,,"left",effect,,,,,,,
	"Все для ремонта и обслуживания","../../../rs/rem.php",,,1,
	"Техподдержка на CD-ROM","../../../support.php",,,1
	]

	menu15=[
	,,170,1,"",prop1,,"left",effect,,,,,,,
	"Лазерные головки","../../../lh/lh.php",,,1,
	"Инверторы подсветки для LCD панелей","../../../data/invertled.php",,,1,
	"Запчасти для СВЧ-печей","",,,1
	]

	menu16=[
	,,170,1,"",prop1,,"left",effect,,,,,,,
	"Гостевая книга","../../../guest/guestbook.php",,,1
	]

	menu17=[
	,,170,1,"",prop1,,"left",effect,,,,,,,
	"Резисторы подстроечные","../../../data/vatronics.php",,,1,
	"Танталовые конденсаторы","../../../data/tantal.php",,,1,
	"EPCOS - ферритовые сердечники","../../../table.php?name=ferrite&page=0",,,1,
	"Силовые трансформаторы 15 и 18 Ватт","../../../table.php?name=transpower&page=0",,,1,
	"Электролитические конденсаторы Hitachi AIC","../../../data/hit-aic/hit-aic.php",,,1,
	"Катушки индуктивности","../../../data/sumida/sumida.php",,,1,
	"Клавиатуры","../../../data/keyboard/key.php",,,1,
	"Вентиляторы","../../../data/fun/fun.php",,,1

	]

	menu18=[
	,,270,1,"",prop1,,"left",effect,,,,,,,
	"Портативный осциллограф 10 МГц - HPS-10","../../../data/pribor/hps-10.php",,,1,
	"Портативный осциллограф 40 МГц - HPS-40","../../../data/pribor/hps-40.php",,,1,
	"USB Осциллограф PV6501","../../../data/pv6501.php",,,1,
	"Измерительные приборы","../../../table.php?name=pribor&page=0",,,1,	
	"Щупы для осциллографа","../../../data/hp.php",,,1,
	"Щупы для измерительных приборов","../../../table.php?name=10510&page=0",,,1,
	"Весы портативные","../../../data/ps-x00.php",,,1,
	"Измеритель вибрации","../../../data/ivpa.php",,,1,
	"Цифровой термометр ETP-104","../../../data/s-line/etp-104.php",,,1,
	"Терморегулятор ТП300","../../../data/tp300.php",,,1,
	"Лазерные дальномеры","../../../data/nivelir/nivelir.php",,,1
	]

	menu19=[
	,,250,1,"",prop1,,"left",effect,,,,,,,
	"Новинки! GSM-модемы","../../../data/fsu.php",,,1,
	"GSM-W1 ОХРАННАЯ GSM-система","../../../data/gsm-w1.php",,,1,
	"Промышленные радиомодемы","../../../data/zigbee/zigbeemodem.php",,,1,
	"Радиомодемы для систем телеметрии","../../../data/saturn.php",,,1,
	"GSM/UMTS/GPS - аксессуары","../../../data/gsmaccessories/antgsm.php",,,1,
	"G P S - навигация","../../../data/gps/gps.php",,,1,
	"BlueTooth модули","../../../data/bluetooth/bluetooth.php",,,1,
	"Промышленные BlueTooth модули компании SENA","../../../data/bluetooth/sena.php",,,1,
	"Передатчики (приемники) диапазона 433 МГц с частотной модуляцией","../../../data/telecontroli/telecontroli.php",,,1,
	"ZigBee-трансиверы","../../../data/chipcon/cc1000.php",,,1,
	"Каталоги по беспроводным компонентам","../../../data/gsmmodem/gsmcatalog.php",,,1
	]

	menu20=[
	,,250,1,"",prop1,,"left",effect,,,,,,,
	"Разъемы DIN 41612","../../../data/41612/?name=612b",,,1,
	"Промышленные клемники","../../../data/klemnik/klem.php",,,1,
	"Держатели предохранителей","../../../table.php?name=1040112&page=0",,,1,
	"Держатели карт памяти","../../../data/memorycard/CCM05-5501LFT-R901.php",,,1,
	"Сверхплоские шлейфы","../../../data/ffc.php",,,1,
	"GAINTA -  высококачественные корпуса для РЭА","../../../table.php?name=1040602&page=0",,,1,
	"Батарейные отсеки и держатели аккумуляторов","../../../data/bathold.php",,,1
	]

