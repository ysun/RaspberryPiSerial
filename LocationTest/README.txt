Polaris控制系统UI使用说明

一、桌面应用版
        使用NW.js开发，NW.js （原名 node-webkit）是一个基于 Chromium 和 node.js 运行的，通过它可以用 HTML 和 JavaScript 编写原生应用程序。它还允许从 DOM 调用 Node.js 的模块 ，实现了一个用所有 Web 技术来写原生应用程序的新的开发模式。 
       在/home/work/LocationTest/App路径下有3个文件分别是index.heml、package.json、TL.js.
index.heml显示前端桌面
package.json设置窗口的相关属性
TL.js包含相关后端的操作函数
启动应用只要在/home/work/LocationTest/App路径下执行nw .命令即可。

二、网页版
        使用html+js开发,在/home/work/LocationTest/Web路径下有4个文件分别是index.js、requestHandlers.js、 router.js 、server.js.
index.js主文件,调用其他模块
requestHandlers.js包含一些请求处理函数
router.js创建路由
server.js创建服务器
       启动服务器只要在/home/work/LocationTest/Web路径下执行node index.js命令即可。然后在浏览器中输入对应网址，端口号为8888.