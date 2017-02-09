//console.log('console TL.js');


//获取头盔输入的坐标值
function getHelmetCoordinate()
{
    var v0 = document.getElementById("textfiled0").value;
    var v1 = document.getElementById("textfiled1").value;
    var v2 = document.getElementById("textfiled2").value;
    var v3 = document.getElementById("textfiled3").value;
    var result=v0+','+v1+','+v2+','+v3;
    return result;
}

//获取手柄输入的坐标值
function getJoystickCoordinate()
{
    var v4 = document.getElementById("textfiled4").value;
    var v5 = document.getElementById("textfiled5").value;
    var v6 = document.getElementById("textfiled6").value;
    var v7 = document.getElementById("textfiled7").value;
    var result1=v4+','+v5+','+v6+','+v7;
    return result1;
}

//获取头盔输入的是绝对坐标还是相对坐标
function getHelmetValue(){  
        var value='';  
        var radio = document.getElementsByName("helmet");  
        for(var i = 0;i<radio.length;i++)  
        {  
            if(radio[i].checked==true)  
            {value = radio[i].value;  
            break;}  
        }  
   return value;  
    }

//获取手柄输入的是绝对坐标还是相对坐标
function getJoystickValue(){  
        var value='';  
        var radio = document.getElementsByName("joystick");  
        for(var i = 0;i<radio.length;i++)  
        {  
            if(radio[i].checked==true)  
            {value = radio[i].value;  
            break;}  
        }  
     return value;  
    }  

//将输入的头盔坐标添加到待执行区域
function Add1()
{	
       	var value=getHelmetValue()+","+getHelmetCoordinate();
        document.getElementById("textarea").value+=value;
        document.getElementById("textarea").value+="\n";
}

//将输入的头盔坐标添加到待执行区域
//function Add2()
//{
	   
//}

//执行待执行区域的命令
function Test()
{
	var loop = document.getElementById("textfiled8").value;
	    var value=document.getElementById("textarea").value;
	        var val=value.split('\n');
		var i=0;
		var j=0;
		for(j=0;j<loop;j++)
		for(i=val.length-1;i>0;i--)
		{
			 var exec = require('child_process').exec; 
			  var cmdStr = '/home/work/control/control'+' '+'-l'+' '+val[i-1];
			  //document.writeln(cmdStr);
			  exec(cmdStr, function(err,stdout,stderr){
				  if(err) {
					console.log('stderr',stderr);
					  } else {
						console.log(stdout);
						}
			  }); 
		}
}

//执行单条头盔命令
function Test1()
{
	    var helmetcoordinate=getHelmetCoordinate();
	    var joystickcoordinate=getJoystickCoordinate();
            var helmetvalue=getHelmetValue();
	    var joystickvalue=getJoystickValue();

            var helmet=helmetvalue+','+helmetcoordinate;
	    var joystick=joystickvalue+','+joystickcoordinate;
	    var exec = require('child_process').exec; 
	    var cmdStr = '/home/work/control/control'+' '+'-l'+' '+helmet;
	   // document.writeln(cmdStr);
	    exec(cmdStr, function(err,stdout,stderr){
		if(err) {
			console.log('stderr',stderr);
				} else {
					console.log(stdout);
					}
				  });
				      
}

//执行单条手柄命令
//function Test2()
//{

//}
