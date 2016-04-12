//  name: tftprotocol.h
//  auther: Chentao Zhao
//  function: define the CTP protocol for the communication between 2560 and TFT touch module.
//  Ctrl, Qury and Evnt are defined base on the GUI menu .
// Ctrl means host send ->msg to module
// Qury means host send -> and get reply <-from module
// Evnt means host recieve <- from 

#ifndef tftprotocol_H
#define tftprotocol_H

#include "Configuration.h"
#include "Marlin.h"


//for debugging
//use uart0 for debug as MYSERIAL
//use uart2 for tft as GSERIAL

//#define xserial MYSERIAL
#define xserial GSERIAL
#define ECHO MYSERIAL.println

// SD list update
//#define qmark "\""


#define TX_INTERNAL 10
#define QRY_INTERNAL 25
#define MNT_INTERNAL 5
#define EINTERVAL 3


#define rxfinger 10
#define rxthumb 10

#define TFT_MSG_B_READY2GO "E+sys.state=ready\r"
//#define TFT_MSG_Q_SELECTEDNAME "Q+sdfsdfd\r"
#define TFT_MSG_V_ENTERSD "E+main_button.state=0\r"
#define	TFT_MSG_V_STARTPRINT "E+sd_button.state=0\r"

#define	TFT_MSG_V_PAUSEPRINT "E+proc_button_4.state=1\r"
#define	TFT_MSG_V_RESUMEPRINT "E+proc_button_3.state=1\r"
#define TFT_MSG_V_STOPPRINT "E+EXITFROMPRINT\r"

#define TFT_MSG_V_ABANDONR "E+ABANDONR\r"

#define TFT_MSG_V_PRINTOVER "E+PRINTOVER\r"

// single axis page verboses are defined here
#define TFT_MSG_A_AXIS_UP "AU\r"
#define TFT_MSG_A_AXIS_DOWN "AD\r"
#define TFT_MSG_A_AXIS_XLEFT "AXL\r"
#define TFT_MSG_A_AXIS_XRIGHT "AXR\r"
#define TFT_MSG_A_AXIS_YLEFT "AYL\r"
#define TFT_MSG_A_AXIS_YRIGHT "AYR\r"

#define TFT_MSG_S_AXIS_STEP1 "S+H+STEP\r"
#define TFT_MSG_S_AXIS_STEP2 "S+L+STEP\r"
//filament filling and retracting 

#define TFT_MSG_V_filafil_H "E+i+START\r"
#define TFT_MSG_V_filaret_H "E+o+START\r"
#define TFT_MSG_V_fila_E "E+FSTOP\r"
#define TFT_MSG_V_fila_COOL "E+FCOOL\r"
//Auxilary button
#define TFT_MSG_F_SHUT "F+SHUTDOWN\r"
#define TFT_MSG_F_LEVEL "F+LEVELING\r"
#define TFT_MSG_F_AXISHOME_SET "F+SETHOME\r"
#define TFT_MSG_F_AXIS_HOME "F+AXISHOME\r"
#define TFT_MSG_F_XHOME "F+ONEAXISHOMEX\r"
#define TFT_MSG_F_YHOME "F+ONEAXISHOMEY\r"
#define TFT_MSG_F_ZHOME "F+ONEAXISHOMEZ\r"
#define TFT_MSG_F_COOLDOWN "F+COOLDOWNSTART\r"
#define TFT_MSG_F_COOLDONE "F+COOLDOWNABORT\r"
#define TFT_MSG_F_PREHEAT "F+PREHEATSTART\r"
#define TFT_MSG_F_PREHEATDONE "F+PREHEATABORT\r"	
#define TFT_MSG_Q_STOP "Q+1\r"
#define TFT_MSG_S_DEBUG "D+SAYHI\r"
// manual leveling
#define TFT_MSG_L_A "L+LEVELINGA\r"
#define TFT_MSG_L_B "L+LEVELINGB\r"
#define TFT_MSG_L_C "L+LEVELINGC\r"
#define TFT_MSG_L_Z "L+LEVELINGZ\r"
#define LEVEL_AX 125
#define LEVEL_AY 132
#define LEVEL_BX 208
#define LEVEL_BY 14
#define LEVEL_CX 36
#define LEVEL_CY 14


//#define TFT_MSG_F_RESUME "F+STARTRESUME\r"



//#define hr(timevalue) timevalue/3600000%24 
//#define mns(timevalue) timevalue/60000%60
//#define snd(timevalue) timevalue/1000%60





//#define tft_hello xserial.println("hello world")


#define tft_sd_detected \
xserial.print("sd_detected()\r");

#define tft_sd_released \
xserial.print("sd_released()\r");

//#define tft_sd_dir(index,filename) xserial.println("sd_comboBox.addOption("#filename","#index")")



//#define tft_sd_add(index,filename)\
//xserial.print("sd_comboBox.addOption(\"");\
//xserial.print(filename);\
//xserial.print("\",");\
//xserial.print(index);\
//xserial.print(")\r")


#define tft_sd_del \
xserial.print("filesys.splice(0,filesys.length)\r");



//#define tft_getfilename \
//xserial.print("sd_comboBox.currentText?\r");

//#define tft_getstopstates \
//xserial.print("stopbutton?\r");

//#define tft_chkconnect \
//xserial.print("connected=1\r");

#define tft_sd_add(filename)\
xserial.print("filesys.push(\"");\
xserial.print(filename);\
xserial.print("\")\r")

#define echo_sd_add(filename)\
ECHO("filesys.push(\"");\
ECHO(filename);\
ECHO("\")\r")

#define echo_sayhi \
ECHO("hi, 2560 is ready to rx :-)");

//#define tft_sd_ready while(1){\
//delay(1000);\
//xserial.write("proc_progressBar.value=58\r");}
//xserial.flush();\
//xserial.write("print_progressBar.value=58\r")
//#define tft_sd_ready xserial.println("sd_comboBox.visible=1;");\
//xserial.println("sd_label1.visible=0;")

//#define tft_update xserial.debugtft()

//#define uart2ready xserial.hearfromtft().compareTo(TFT_MSG_B_READY2GO)

#define	tft_thermo_set(value)\
xserial.print("update_t(");\
xserial.print(value);\
xserial.print(")\r")

#define	tft_pos_qt_set(xvalue,yvalue,fvalue)\
xserial.print("update_xy(");\
xserial.print(xvalue);\
xserial.print(",");\
xserial.print(yvalue);\
xserial.print(",");\
xserial.print(fvalue);\
xserial.print(")\r")

#define	tft_progressbar_set(value)\
xserial.print("update_p(");\
xserial.print(value);\
xserial.print(")\r")

#define	tft_leveling_enter \
xserial.print("enterlevel()\r");\
MYSERIAL.print("enterlevel()\r")

#define	tft_heatingloop_enter \
xserial.print("enterheatingloop()\r");\
MYSERIAL.print("enterheatingloop()\r")

#define	tft_printing_enter \
xserial.print("enterprint()\r");\
MYSERIAL.print("enterprint()\r")

#define	tft_sys_start \
xserial.print("systemstart()\r")

#define tft_sys_welcome \
xserial.print("welcomepage()\r");

#define	tft_filamove_enter \
xserial.print("enterfilaex()\r");\
MYSERIAL.print("enterfilaex()\r")

#define	tft_sd_memory_fail \
xserial.print("sdmemoryfail()\r");\
MYSERIAL.print("sdmemoryfail()\r")

#define	tft_heatingloop_set \
xserial.print("enterheatingloop()\r");\
MYSERIAL.print("enterheatingloop()\r")

#define	tft_heatingloop_clear \
xserial.print("heatingloop=false\r");\
MYSERIAL.print("heatingloop=false\r")

#define	tft_resume_enable \
xserial.print("resumeon()\r");\
MYSERIAL.print("resumeon()\r")

#define	tft_resume_disable \
xserial.print("resumeoff()\r");\
MYSERIAL.print("resumeoff()\r")

#define	tft_printing_over \
xserial.print("printingover()\r")

#define	tft_flash_write \
xserial.print("flashwrite()\r")

#define	tft_light_on \
xserial.print("lighton()\r")

#define	tft_light_off \
xserial.print("lightoff()\r")

#define tft_resume_valid \
xserial.print("resume_valid()\r");

#define tft_pause_done \
xserial.print("printpause()\r");

#define	tft_sdin \
xserial.print("sdin()\r")

#define	tft_sdout \
xserial.print("sdout()\r")

#define	tft_total_time(value)\
xserial.print("lefttime=");\
xserial.print(value);\
xserial.print("\r")


#define	tft_console_set(cvalue,bvalue,pvalue)\
xserial.print("console_set(");\
xserial.print(cvalue);\
xserial.print(",");\
xserial.print(bvalue);\
xserial.print(",");\
xserial.print(pvalue);\
xserial.print(")\r")


/*	
#define	tft_time_set(value)\
MYSERIAL.write("proc_label5.text=\"");\
MYSERIAL.print(hr(value));\
MYSERIAL.write(":");\
MYSERIAL.print(mns(value));\
MYSERIAL.write(":");\
MYSERIAL.print(snd(value));\
MYSERIAL.write("\"\r");

#define	tft_lefttime_set(value)\
MYSERIAL.write("proc_label6.text=\"");\
MYSERIAL.print(hr(value));\
MYSERIAL.write(":");\
MYSERIAL.print(mns(value));\
MYSERIAL.write(":");\
MYSERIAL.print(snd(value));\
MYSERIAL.write("\"\r");\
ECHO()



#define uart2available xserial.available()
*/
//#define tftconnected (xserial.hearfromtft().compareTo(TFT_MSG_B_READY2GO)>0)?true:false

//#define fileselected xserial.hearfromtft()(xserial.hearfromtft().compareTo(TFT_MSG_B_READY2GO)>0)?true:false

//#define tft_started xserial.available()&&strcmp(tft_rx_buffer,"E+sys.state=ready")

//readString

#define qkstart4debug card.getfilename(0);\
monitormode=true;\
char* lowname=&card.filename[0];\
for(int8_t i=0;i<strlen(lowname);i++) lowname[i]=tolower(lowname[i]);\
ECHO(lowname);\
char cmd[30];\
sprintf_P(cmd, PSTR("M23 %s"), lowname);\
MYSERIAL.println(cmd);\
enquecommand(cmd);\
enquecommand_P(PSTR("M24"))



//rxmode value illustrator
#define TFTCONNECTED 0
#define FILESELECTED 1 // should be unique because the sd selected process will be perform when rx returned 1;
#define PRINTSTOPED 2
#define PRINTRESUMED 2
#define PRINTPAUSED 2
#define NOVERBOSE -1
#define RESPONSED 2
#define AXISMOVING 3
#define AXISCHANGED 3
#define ACCRYCHANGED 3
#define FILAOPERATE 3
#define AUXFCTDONE 4
#define SETTINGDONE 4

//query type 
#define queryfilename 0
#define querystop 2

//AXIS TYPE
#define steplength1 2.0
#define steplength2 10.0
#define steplength3 30.0

//Extruder move type 
#define nomove 0
#define fillmove 1
#define retrmove 2

//
//#define pwroff_re_proc \
//invalid4resume=true;\
//enquecommand_P(PSTR("G93"));\
//enquecommand_P(PSTR("M157"));\
//enquecommand_P(PSTR("M160"))

//invalid4resume=false


#endif




