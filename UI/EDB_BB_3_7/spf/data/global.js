    // main timer
 var main_timerset=false;
// proc ��ص�ȫ�ֱ���
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

// �����˶���ص�ȫ�ֱ���
  var axis2move="Z";
  var pulsenum=9;
  var direction="UP";
  var stepinfo="L";
  var steplength=2;
  var displace_acc=0;
  var x_old=0;
  var y_old=0;
  var z_old=0;
// ������صĻ�������
  var filaop="i";
// SD ���ļ�����ȫ�ֱ�����main���������ڸ��£��ڴ�ӡ�����У�2560���ٷ��͸���
// sd detected should be updated by the 2560 Tx action.
 // var sd_detected=0;
  //var sd_released=0;
  var sd_states=0;
// ����SD�����.gcode �ļ���Ŀ¼��ʾ
  var filesys=new Array();
  var filename="";
  var fileidx=0;
  var page=1;
// ʱ��ϵͳ�������ϵ�����ϵͳ,��ӡ���Զ��ػ�
  var acctime=new Number(sysVariable.read("acctime_n"));
  var releasedate="2015��7��24��";
  var resumevalid=0;
  var stbstopvalid=sysVariable.read("stbstopvalid_n");
  var stbstoptime=0;
  var STBTIMESET="600"; //10 min * 60
  var IDLETIMESET=sysVariable.read("IDLETIMESET_n");

 //�������־λ
  var toolplace=0; // toolbox open in different place, return where it is
  //�Ż����Ա�־λ
 // var PASSWORDSET="xaffs230"; // ����
  //var FEEDRATE=1; // for the x-y top view imitation calibration
  var SD_MOVERATE=50;// the rate of the MOVE of the file ,the smaller, the faster the file will move

  var Z_MIN=0;
  var Z_MAX=205;
  var Y_MIN=0;
  var Y_MAX=210;
  var X_MIN=0;
  var X_MAX=260;


  // ������ʼ��
  var factory_set=sysVariable.read("FACTORYSET_n");
  //���´�ӡ��״̬

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
        proc_button_3.visible=false;//������ť
        proc_button_4.visible=true;//��ͣ��ť
        //proc_image_7.visible=true; //��ť����
        //��ӡ��ʼ�������������¼��ӡ���ļ����Ͷϵ������־��Ϊ�ϵ�������׼����
        //changeconform_label.text="��ȷ���Ƿ�ֹͣ��ǰ����".concat(filesys[fileidx]);
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
        proc_button.enabled=false;//��ͣ��ť
        proc_button_4.visible=false;//������ť
        proc_button_3.visible=false;//��ͣ��ť
        //proc_image_7.visible=false;//��ͣ������ť
        proc_button_6.visible=true;//������ҳ
        proc_label_20.text="�����";
        proc_image_8.visible=false;
        proc_label_3.visible=false;
        proc_image_6.source='bree.gif';
        proc_image_6.stop();
        sysVariable.write("acctime_n", acctime);
        stbstoptime=0; // ��λ�Զ��ػ���ʱ��
        main_timer.run(0);
  }
    function printini()
  {     //proc initial here
        main_timer.stop();
        proc_image_6.stop();
        B_printing=true;
        acctime=sysVariable.read("acctime_n");
        proc_button_3.visible=false;//��ͣ��ť
        proc_button_4.visible=false;//������ť
       // proc_image_7.visible=false; //��ť����
        proc_image_10.resize(191,86);// process bar initiating
        proc_image_3.move(172,70);// ����ͼ initiating
        proc_image_2.move(30,80);// ����ͼ initiating
       // proc_label_11.styleSheet = "color:black"; //��ӡʱ�� ��
       // proc_label_12.styleSheet = "color:black";//ʣ��ʱ�� ��
        proc_label_20.text="���ڴ�ӡ�ļ�<".concat(filesys[fileidx],">");
        proc_button_6.visible=false;// ���ذ�ť����
        proc_button.visible=true; // ��ͣ��ť��Ч
        proc_button.enabled=true; // ��ͣ��ť��Ч
        proc_button_5.visible=false;
        proc_label_3.visible=false;// ��������ʾ����
        proc_image_8.visible=false;// ��������ʾ����
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
            sd_image_5.visible=true;//�ļ�ͼƬ
            sd_image.visible=true;//U��ͼƬ
            sd_label_3.visible=true;// ѡ����ļ���
            sd_machine.visible=false;
            sd_label_4.visible=true; // �ļ��б�
            sd_label_2.visible=true; // �ļ��б�
            sd_label.visible=false; //���� �����SD��
            sd_label_4.text="...\n";
            for(var i=0;i<((filesys.length<9)?filesys.length:8);i++){
                sd_label_4.text=sd_label_4.text.concat(filesys[i].substring(0,filesys[i].lastIndexOf(".gcode")),"\n");
            }

            for(var i=0;i<5000;i++);//�ȴ�SD���������ļ���ȫ������ٲ�����
            sd_label_3.text=(filesys.length>0)?filesys[0].substring(0,filesys[0].lastIndexOf(".gcode")):"û�п���ʹ�õ�gcode";
            sd_button.visible=(filesys.length>0)?true:false;//��һ���ļ���ť
            sd_button_3.visible=(filesys.length>0)?true:false;//��һ���ļ���ť
            sd_image_8.visible=(filesys.length>0)?true:false; //sd��Ŀ¼�б������
            sd_button_4.visible=(filesys.length>0)?true:false;//ѡ���ļ���ť

            sd_image_9.visible=(filesys.length>8)?true:false; // sd��Ŀ¼��ҳ������
            sd_button_5.visible=(filesys.length>8)?true:false;// sd��Ŀ¼��ҳ��ť
            sd_button_6.visible=(filesys.length>8)?true:false;// sd��Ŀ¼��ҳ��ť
            sdconfirm_label_2.text="".concat(filesys[fileidx],"?");
            // ��ҳ����
            homepage_label_10.text="׼������";//��ҳ��SD��״̬
            sd_states=1;
            //proc_label_20.text="sd������";// ����̨��״̬��ʾ



     }
     function sd_released()
     {
            sd_image_5.visible=false;//�ļ�ͼƬ
            sd_image.visible=false;//U��ͼƬ
            sd_button_4.visible=false;//��ӡ�ļ���ť
            sd_label_3.visible=false; // ѡ����ļ���
            sd_machine.visible=true;
            sd_label_4.visible=false; // �ļ��б�
            sd_label_2.visible=false; // �ļ��б�
            sd_label.visible=true; //��ʾ �����SD��
            sd_button.visible=false;//��һ���ļ���ť
            sd_button_3.visible=false;//��һ���ļ���ť
            sd_image_8.visible=false; //sd��Ŀ¼�б������
            sd_image_9.visible=false; // sd��Ŀ¼��ҳ������
            sd_button_5.visible=false;// sd��Ŀ¼��ҳ��ť
            sd_button_6.visible=false;// sd��Ŀ¼��ҳ��ť
            homepage_label_10.text="δ�忨";//��ҳ��SD��״̬
            proc_label_20.text="sd������";// ����̨��״̬��ʾ
            sd_states=0;
            page=1;// ��ʼ��ҳ����
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

    proc_button.visible=false;// ��ͣ�������أ�ֱ��������ӡ��
    //proc_button_3.visible=true;// ������ӡ���֣� resume_valid()
    //proc_button_5.visible=true;// С������֣� resume_valid()
    //proc_label_11.styleSheet="color:red";
    //proc_label_12.styleSheet="color:red";
    //proc_label_20.text="��ͣ��";
    proc_timer.stop();
    //xpos_mem=xpos;
    //ypos_mem=ypos;
    update_xy(172,70,2000);
}
function resume_valid()
{

    proc_button_3.visible=1;//resume button
    proc_button_5.visible=1;//tool box
    proc_label_20.text="����ͣ�����̼���������������빤����";
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
preheat_lcdNumber.value=t;//Ԥ�ȵ��¶���ʾ
cooldown_lcdNumber.value=t;//��ȴ���¶���ʾ
idling_lcdNumber.value=t;//�������¶���ʾ
fila_Thermo.value=t;//�����ϵ��¶���ʾ
proc_lcdNumber.value=t;  //������̨���¶���ʾ
if(t>55&&(homepage_label.visible==0)){homepage_label.visible=1;}
if(t<55&&(homepage_label.visible==1)){homepage_label.visible=0;}
}

function update_p(p){
        sysCom0.clearReadBuffer();// �������ȵ�ʵʱ��Ҫ��һ�����½��ȣ��������Rx���档


        prs=p;
        //proc_lcdNumber_2.value=p;
        //proc_image_6.speed=2*p+100;

        proc_image_10.move(258+Math.floor(p*191/100),90);//258,90
        proc_image_10.resize(Math.ceil((100-p)*191/100),86);//191,86
  }
function sdmemoryfail(){

}
