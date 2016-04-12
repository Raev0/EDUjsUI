    // main timer
 var main_timerset=false;
// proc 相关的全局变量
  var heatingloop=0;
  var leveling=0;
  var B_printing=false;
  var printtime;
  var lefttime=0;
  var hours;
  var minutes;
  var seconds;
  var hours_l;
  var minutes_l;
  var seconds_l;

//var scanrate=30;
//  var xpos=172;
//  var ypos=70;
  var xpos_mem=0;
  var ypos_mem=0;
  var deltax;
  var deltay;
  var stepx=0;
  var stepy=0;
  var stored_p=0;
//  var t=0;
  var prs=0;
//  var f; // feedrate , determine the speed the x and y travels
//  var f_update=0;
  var F_BASE=75;

// 单轴运动相关的全局变量
  var axis2move="Z";
  var pulsenum=9;
  var direction="UP";
  var stepinfo="L";
  var steplength=2;
  var displace_acc=0;
  var x_old=0;
  var y_old=0;
  var z_old=0;
// 进料相关的环境变量
  var filaop="i";
// SD 卡的检测相关全局变量，main计数器定期更新，在打印过程中，2560不再发送更新
// sd detected should be updated by the 2560 Tx action.
 // var sd_detected=0;
  //var sd_released=0;
  var sd_states=0;
// 管理SD卡里的.gcode 文件和目录显示
  var filesys=new Array();
  var filename="";
  var fileidx=0;
  var page=1;
// 时间系统，包括断点续打系统,打印完自动关机
  var acctime=new Number(sysVariable.read("acctime_n"));
  var releasedate="2015年7月24日";
  var resumevalid=0;
  var stbstopvalid=sysVariable.read("stbstopvalid_n");
  var stbstoptime=0;
  var STBTIMESET="600"; //10 min * 60
  var IDLETIMESET=sysVariable.read("IDLETIMESET_n");

 //工具箱标志位
  var toolplace=0; // toolbox open in different place, return where it is
  //优化调试标志位
 // var PASSWORDSET="xaffs230"; // 密码
  //var FEEDRATE=1; // for the x-y top view imitation calibration
  var SD_MOVERATE=50;// the rate of the MOVE of the file ,the smaller, the faster the file will move

  var Z_MIN=0;
  var Z_MAX=205;
  var Y_MIN=0;
  var Y_MAX=210;
  var X_MIN=0;
  var X_MAX=260;


  // 出厂初始化
  var factory_set=sysVariable.read("FACTORYSET_n");
  //更新打印的状态

  function gif_all_pause()
  {

         idling_image.stop();
         proc_image_7.stop();
         proc_image_6.stop();
         guide_change_image.stop();
         load_image.stop();
         load_image_2.stop();


  }
  function lefttimeshow()
  {
        proc_label_6.visible=true;
        proc_label_12.visible=true;
        proc_label_13.visible=true;
        proc_label_14.visible=true;
        proc_label_15.visible=true;
        proc_label_16.visible=true;

  }

  function lefttimeshade()
  {
        proc_label_6.visible=false;
        proc_label_12.visible=false;
        proc_label_13.visible=false;
        proc_label_14.visible=false;
        proc_label_15.visible=false;
        proc_label_16.visible=false;

  }
  function enterlevel()
  {
      lefttimeshade();
      proc_timer.interval=750;
      proc_button.visible=false;
      proc_image_8.visible=false;
      proc_label_3.visible=false;
      leveling=true;
      heatingloop=false;
  }


  function enterprint()
  {
        lefttimeshow();
        leveling=false;
        proc_button.visible=true;
        proc_timer.interval=1000;
        heatingloop=false;
        proc_label_3.visible=false;
        proc_image_8.visible=false;
        proc_button_3.visible=false;//继续按钮
        proc_button_4.visible=true;//暂停按钮
        //proc_image_7.visible=true; //按钮背景
        //打印初始化定义在这里，记录打印的文件名和断点续打标志，为断点续打做准备。
        //changeconform_label.text="请确认是否停止当前任务".concat(filesys[fileidx]);
        sysVariable.write("resumefile_n", filesys[fileidx]);
        sysVariable.write("resumevalid_n", 1);
        sysVariable.write("fileidx_n", fileidx);
        proc_image_6.play(0);



  }

  function enterheatingloop()
  {
      lefttimeshade();
      proc_label_19.visible=false;
      proc_button.visible=true;
      proc_timer.interval=500;
      leveling=false;
      heatingloop=true;
  }
  function enterfilaex()
  {

        guide_load.show("fade-in");
        fila_button_5.enabled=true;
        fila_button_6.visible=true;
        fila_button_6.enabled=true;
        fila_button_5.visible=true;

        fila_image_2.visible=false;
        fila_image.visible=false;
        fila_Thermo.visible=false;
        fila_image_3.visible=false;
        fila_image_4.visible=false;
        fila_image_5.visible=false;
        fila_button_7.visible=false;
  }
  function welcomepage()
  {
            main_label_2.visible=false;
            main_label_3.visible=true;
            sysBacklight.brightness=(sysBacklight.brightness==8)?8:sysVariable.read("brightness_n");
            gif_all_pause();

  }

 function systemstart()
 {
         main_timer.run(0);
         homepage.show("fade-in");
         if(factory_set!="set")
             homepage_button_4.visible=true;

  }
    function printingover()
  {
        heatingloop=0;
        leveling=1;
        proc_timer.stop();
        update_p(100);
        B_printing=false;
        //main_timer.interval=100;
        //proc_lcdNumber_2.value=100;
        proc_button.enabled=false;//急停按钮
        proc_button_4.visible=false;//继续按钮
        proc_button_3.visible=false;//暂停按钮
        //proc_image_7.visible=false;//暂停继续按钮
        proc_button_6.visible=true;//返回首页
        proc_label_20.text="已完成";
        proc_image_8.visible=false;
        proc_label_3.visible=false;
        proc_image_6.source='bree.gif';
        proc_image_6.stop();
        sysVariable.write("acctime_n", acctime);
        stbstoptime=0; // 复位自动关机计时器
        main_timer.run(0);
  }
    function printini()
  {     //proc initial here
        main_timer.stop();
        proc_image_6.stop();
        B_printing=true;
        acctime=sysVariable.read("acctime_n");
        proc_button_3.visible=false;//暂停按钮
        proc_button_4.visible=false;//继续按钮
       // proc_image_7.visible=false; //按钮背景
        proc_image_10.resize(191,86);// process bar initiating
        proc_image_3.move(172,70);// 俯视图 initiating
        proc_image_2.move(30,80);// 俯视图 initiating
       // proc_label_11.styleSheet = "color:black"; //打印时间 秒
       // proc_label_12.styleSheet = "color:black";//剩余时间 秒
        proc_label_20.text="正在打印文件<".concat(filesys[fileidx],">");
        proc_button_6.visible=false;// 返回按钮隐藏
        proc_button.visible=true; // 急停按钮有效
        proc_button.enabled=true; // 急停按钮有效
        proc_button_5.visible=false;
        proc_label_3.visible=false;// 加热中提示隐藏
        proc_image_8.visible=false;// 加热中提示隐藏
        //proc_image_6.speed=100;
        lefttimeshow();
        proc_image_7.pause(1);
        proc_image_7.source='procRing3.gif';



  }
    function resumeon()
  {
        resumevalid=1;
        sysVariable.write("resumevalid_n",resumevalid);
  }
     function resumeoff()
  {
        resumevalid=0;
        sysVariable.write("resumevalid_n",resumevalid);
  }
     function sd_detected()
     {
            sd_image_5.visible=true;//文件图片
            sd_image.visible=true;//U盘图片
            sd_label_3.visible=true;// 选择的文件名
            sd_machine.visible=false;
            sd_label_4.visible=true; // 文件列表
            sd_label_2.visible=true; // 文件列表
            sd_label.visible=false; //隐藏 请插入SD卡
            sd_label_4.text="...\n";
            for(var i=0;i<((filesys.length<9)?filesys.length:8);i++){
                sd_label_4.text=sd_label_4.text.concat(filesys[i].substring(0,filesys[i].lastIndexOf(".gcode")),"\n");
            }

            for(var i=0;i<5000;i++);//等待SD卡的所有文件完全载入后再操作。
            sd_label_3.text=(filesys.length>0)?filesys[0].substring(0,filesys[0].lastIndexOf(".gcode")):"没有可以使用的gcode";
            sd_button.visible=(filesys.length>0)?true:false;//上一个文件按钮
            sd_button_3.visible=(filesys.length>0)?true:false;//下一个文件按钮
            sd_image_8.visible=(filesys.length>0)?true:false; //sd卡目录列表滚动条
            sd_button_4.visible=(filesys.length>0)?true:false;//选择文件按钮

            sd_image_9.visible=(filesys.length>8)?true:false; // sd卡目录翻页滚动条
            sd_button_5.visible=(filesys.length>8)?true:false;// sd卡目录翻页按钮
            sd_button_6.visible=(filesys.length>8)?true:false;// sd卡目录翻页按钮
            sdconfirm_label_2.text="".concat(filesys[fileidx],"?");
            // 首页更新
            homepage_label_10.text="准备就绪";//首页的SD卡状态
            sd_states=1;
            //proc_label_20.text="sd卡错误";// 监视台的状态显示



     }
     function sd_released()
     {
            sd_image_5.visible=false;//文件图片
            sd_image.visible=false;//U盘图片
            sd_button_4.visible=false;//打印文件按钮
            sd_label_3.visible=false; // 选择的文件名
            sd_machine.visible=true;
            sd_label_4.visible=false; // 文件列表
            sd_label_2.visible=false; // 文件列表
            sd_label.visible=true; //显示 请插入SD卡
            sd_button.visible=false;//上一个文件按钮
            sd_button_3.visible=false;//下一个文件按钮
            sd_image_8.visible=false; //sd卡目录列表滚动条
            sd_image_9.visible=false; // sd卡目录翻页滚动条
            sd_button_5.visible=false;// sd卡目录翻页按钮
            sd_button_6.visible=false;// sd卡目录翻页按钮
            homepage_label_10.text="未插卡";//首页的SD卡状态
            proc_label_20.text="sd卡错误";// 监视台的状态显示
            sd_states=0;
            page=1;// 初始化页码数
     }
     function shutdown()
     {
        sysCom0.write("F","+","S","H","U","T","D","O","W","N","\r","\0");
        sysManager.execute("reboot");
     }
function flashwrite()
{
        sysVariable.write("acctime_n", acctime);
        sysVariable.write("resumetm_n", printtime);
        sysVariable.write("resumeltm_n", lefttime);
        sysVariable.write("resumep_n", p);
}

function lighton()
{
guide_leveling_image_2.visible=true;
guide_leveling_image_3.visible=false;
}
function lightoff()
{
guide_leveling_image_3.visible=true;
guide_leveling_image_2.visible=false;
}


function printpause()
{

    proc_button.visible=false;// 急停开关隐藏，直到继续打印；
    //proc_button_3.visible=true;// 继续打印出现； resume_valid()
    //proc_button_5.visible=true;// 小红箱出现； resume_valid()
    //proc_label_11.styleSheet="color:red";
    //proc_label_12.styleSheet="color:red";
    //proc_label_20.text="暂停中";
    proc_timer.stop();
    //xpos_mem=xpos;
    //ypos_mem=ypos;
    update_xy(172,70,2000);
}
function resume_valid()
{

    proc_button_3.visible=1;//resume button
    proc_button_5.visible=1;//tool box
    proc_label_20.text="已暂停；按绿键继续；按红箱进入工具箱";
    proc_image_6.source='bree.gif';
    proc_image_6.stop();

}

function update_xy(xpos,ypos,f)
{
    deltax=xpos-proc_image_3.x;
    deltay=ypos-proc_image_3.y;
    stepx=0;
    stepy=0;

 if(deltax>0.9){stepx=1;}
 else if (deltax<-0.9){stepx=-1;}//-1*Math.floor(f/2000);}

 if(deltay>0.9){stepy=1;}
 else if(deltay<-0.9){stepy=-1;}

 //if(f_update){// update the feedrate of the movement behind
        proc_timer_2_x.interval=(f==0)?F_BASE:Math.abs(F_BASE*1000*Math.sqrt(deltax*deltax+deltay*deltay)/f/deltax);
        proc_timer_2_y.interval=(f==0)?F_BASE:Math.abs(F_BASE*1000*Math.sqrt(deltax*deltax+deltay*deltay)/f/deltay);
 //       f_update=0;
        //proc_lcdNumber_3.value=f;
        //proc_lcdNumber_4.value=proc_timer_2_x.interval;
        //proc_lcdNumber_6.value=proc_timer_2_y.interval;
  //           }
        proc_timer_2_x.stop();
        proc_timer_2_y.stop();
 if(Math.abs(deltax)>0)
     proc_timer_2_x.run(Math.floor(Math.abs(deltax)));
 if(Math.abs(deltay)>0)
        proc_timer_2_y.run(Math.sqrt(deltax*deltax+deltay*deltay));
        proc_timer_2_y.run(Math.floor(Math.abs(deltay)));
}
function update_t(t)
{
preheat_lcdNumber.value=t;//预热的温度显示
cooldown_lcdNumber.value=t;//冷却的温度显示
idling_lcdNumber.value=t;//待机的温度显示
fila_Thermo.value=t;//进退料的温度显示
proc_lcdNumber.value=t;  //主控制台的温度显示
if(t>55&&(homepage_label.visible==0)){homepage_label.visible=1;}
if(t<55&&(homepage_label.visible==1)){homepage_label.visible=0;}
}

function update_p(p){
        sysCom0.clearReadBuffer();// 提升进度的实时性要求，一旦更新进度，清空所有Rx缓存。


        prs=p;
        //proc_lcdNumber_2.value=p;
        //proc_image_6.speed=2*p+100;

        proc_image_10.move(258+Math.floor(p*191/100),90);//258,90
        proc_image_10.resize(Math.ceil((100-p)*191/100),86);//191,86
  }
function sdmemoryfail(){

}
