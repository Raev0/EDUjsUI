var i = 0;
var temp=0;
function pid_ini()
{
// Set update region (x, y, width, height)
cc.setActiveRegion(5, 5, cc.width-5, cc.height-5);

// Y axis
cc.createObject("axis_y", "line",5, 0, 5, cc.height);
cc.setObjectPalette("axis_y", "pen", "#FF0000", "solid", 1);
//Y arrow
cc.extendObject("axis_y", 5, 0, 0, 5);
cc.extendObject("axis_y", 5, 0, 10, 5);
// Y scale
for (var i=0; i<(cc.height-15)/10; i++) {
 cc.extendObject("axis_y",  0, cc.height-i*10, 5, cc.height-i*10);
}

// X axis
cc.createObject("axis_x", "line",0, cc.height-6, cc.width, cc.height-6);
cc.setObjectPalette("axis_x", "pen", "#00FFFF", "solid", 1);
// X arrow
cc.extendObject("axis_x", cc.width, cc.height-6, cc.width-5, cc.height-11);
cc.extendObject("axis_x", cc.width, cc.height-6, cc.width-5, cc.height-1);
// X scale
for (var i=0; i<(cc.width-15)/10; i++) {
    cc.extendObject("axis_x",(i+1)*10, cc.height-6, (i+1)*10, cc.height);
}

cc.createObject("myline", "line", 0, 30, 480,30);
cc.setObjectPalette("myline", "pen", "#fff799", "solid", 1);
//cc.createObject("myline2", "line", 0, 200, 480,200);



cc.createObject("youpoly", "polyline",  5, 60*Math.cos(Math.PI /10 * 0) +150);
cc.setObjectPalette("youpoly",  "pen", "#FF0000", "solid", 1);


// Move Point
cc.createObject("mypoint", "point",  10, 40*Math.cos(Math.PI /10 * 0) +150);
cc.setObjectPalette("mypoint", "pen", "#FFFF00", "solid", 8);

cc.setObjectMoving("mypoint", "absolute");

}
function update_t(t)
{
main_lcdNumber.value=t;//预热的温度显示
//main_plot.value = t;
temp=t;
//if(t>main_plot.yAxisScaleMax){
//    main_plot.yAxisScaleMax=t;}

}
function update_xy(xv,yv,zv)
{

}

function update_p(p)
{


}
 function enterheatingloop()
 {

 }
function PID_read(p_para,i_para,d_para)
{
   main_lineEdit_5.text=p_para;
   main_lineEdit_6.text=i_para;
   main_lineEdit_7.text=d_para;
}

function flashwrite()
{

}
function feedrate_read(fdrt)
{
main_lineEdit.text=fdrt;

}
function welcomepage()
{
main_label_2.text="testing handshake...";

}
function systemstart()
{
main_label_2.text="handshake done!";
}
function lighton()
{
main_image_2.visible=true;
main_image.visible=false;
}

function lightoff()
{
main_image_2.visible=false;
main_image.visible=true;
}

function indication(text)
{
main_label_2.text=text;
}

function Z_offset(value)
{
main_lineEdit_2.text=value;
}

function servo_angle(value)
{
main_lineEdit_3.text=value;
}

function machine_series(value)
{   
 var temp=main_lineEdit_4.text.concat(value);
 main_lineEdit_4.text=temp;//main_lineEdit_4.text.concat(value);
}

