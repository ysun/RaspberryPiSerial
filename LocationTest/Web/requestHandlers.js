var querystring = require("querystring");
//var fs = require("fs") ;

function start(response, postData){
  console.log("Request handler 'start' was called.");

var postHTML='<html>'+
  '<head>'+
    '<meta http-equiv="Content-Type" content="text/html; charset=utf-8">'+
    '<title>LocationTest</title>'+
     '<script type="text/javascript">'+
        'function Add1(response,postData)'+
	'{'+
    		'var v0 = document.getElementById("textfiled0").value;'+
    		'var v1 = document.getElementById("textfiled1").value;'+
    		'var v2 = document.getElementById("textfiled2").value;'+
    		'var v3 = document.getElementById("textfiled3").value;'+
    		'var result=v0+","+v1+","+v2+","+v3;'+
    
        	'var value="";'+  
        	'var radio = document.getElementsByName("helmet");'+  
        	'for(var i = 0;i<radio.length;i++)'+  
        	'{'+  
            	'if(radio[i].checked==true)'+  
            	'{value = radio[i].value;'+  
            	'break;}'+  
        	'}'+  

    		'var val=value+","+result+"\\n";'+
    		'document.getElementById("textarea").value+=val;'+
	'}'+

     '</script>'+

  '</head>'+
  '<body>'+
    '<h1 align="center">空间定位测试</h1>'+

'<form action="/helmet" method="post">'+
'<font color="#00F" size="5">头盔</font>'+ 
'<br>'+
'绝对坐标'+
'<input type="radio" name="helmet" id="absolute1" value="A" />'+
'相对坐标'+
'<input type="radio" checked="checked" name="helmet" id="relative1" value="B" />'+
'<br>'+
'X:<input type="text" name="text0" id="textfiled0" size="10" maxlength="20" title="请输入0-1000内的坐标值">'+
'Y:<input type="text" name="text1" id="textfiled1" size="10" maxlength="20">'+
'Z:<input type="text" name="text2" id="textfiled2" size="10" maxlength="20">'+
'W:<input type="text" name="text3" id="textfiled3" size="10" maxlength="20">'+
'<br>'+
'<input type="submit" value="执行" accesskey="h" title="快捷键:Alt+h"/>'+
'<input type="button" value="添加" onclick="Add1();" accesskey="a" title="快捷键:Alt+a"/>'+
'</form>'+
'<br>'+
'<p> </p>'+

'<font color="#00F" size="5">手柄</font>'+ 
'<br>'+
'绝对坐标'+
'<input type="radio" name="joystick" id="absolute2" value="A" />'+
'相对坐标'+
'<input type="radio" checked="checked" name="joystick" id="relative2" value="B" />'+
'<br>'+
'X:<input type="text" name="text4" id="textfiled4" size="10" maxlength="20">'+
'Y:<input type="text" name="text5" id="textfiled5" size="10" maxlength="20">'+
'Z:<input type="text" name="text6" id="textfiled6" size="10" maxlength="20">'+
'W:<input type="text" name="text7" id="textfiled7" size="10" maxlength="20">'+
'<br>'+
'<input type="submit" value="执行" onclick="Test2()" accesskey="j" title="快捷键:Alt+j"/>'+
'<input type="submit" value="添加" onclick="Add2()" accesskey="b" title="快捷键:Alt+b"/>'+
'<br>'+
'<br>'+
'<hr style="height:1px;border:none;border-top:1px solid #555555;" />'+
'待执行命令'+
'<br>'+
'<form action="/test" method="post">'+
'<textarea name="textarea" id="textarea" cols="20" rows="10"></textarea>'+
'循环<input type="text" name="text8" id="textfiled8" size="3" maxlength="20" value="1">次'+
'<br>'+
'<br>'+
'<div align="center"><input type="submit" value="开始执行" accesskey="t" title="快捷键:Alt+u"/></div>'+
'<p>         </p>'+
'</form>'+
  '</body>'+
'</html>';

    response.writeHead(200,{"Content-Type":"text/html"});
    response.write(postHTML);
    response.end();
 }
/*
var pathname1="index.html";

if(pathname1){
 response.writeHead(200, {"Content-Type":"text/html"});
 fs.readFile(pathname1,function (err,data){
 response.end(data);
});
 } else {
 response.writeHead(404, {"Content-Type":"text/html"});
 response.end("<h1>404 Not Found</h1>");
}
*/

function test(response, postData){
  console.log("Request handler 'test' was called.");

  var exec = require('child_process').exec; 
  var loop=querystring.parse(postData).text8;
  var value=querystring.parse(postData).textarea;

  var val=value.split('\n');
  var i=0;
  var j=0;

  for(j=0;j<loop;j++)
  for(i=val.length-1;i>0;i--)
  {
 
   var cmdStr = '/home/work/control/control'+' '+'-l'+' '+val[i-1];

 exec(cmdStr, function(err,stdout,stderr){
	if(err) {
		console.log('stderr',stderr);
	} else {
		console.log(stdout);
	}
}); 
}
 response.writeHead(200,{"Content-Type":"text/plain"});
 response.write("Execution");
 response.end();
}


function helmet(response, postData){
  console.log("Request handler 'helmet' was called.");

//获取各个输入框的值
  var value0=querystring.parse(postData).text0;
  var value1=querystring.parse(postData).text1;
  var value2=querystring.parse(postData).text2;
  var value3=querystring.parse(postData).text3;
  var val1=querystring.parse(postData).helmet;

  var helmet=val1+","+value0+","+value1+","+value2+","+value3;
  var cmdStr = '/home/work/control/control'+' '+'-l'+' '+helmet;
  console.log(cmdStr);

  var exec = require('child_process').exec; 
  exec(cmdStr, function(err,stdout,stderr){
	if(err) {
		console.log('stderr',stderr);
	} else {
		console.log(stdout);
	}
}); 

 response.writeHead(200,{"Content-Type":"text/plain"});
 response.write("Execution");
 response.end();
}


exports.start = start;
exports.test = test;
exports.helmet = helmet;
