# XEMA
 A low cost, full featured open source, industrial 3D camera.

![DFX800](images/DFX804.jpg)

> This project is an Open Source industrial 3D camera. The goal at current stage is to create a low cost 3D camera with capability of current generation industrial cameras. It is also a computer with GPU computing capability, which makes running 3D point cloud recognition algorithms and robotic arm control programs possible. This camera can be used in scenarios such as bin-picking, assembly, 3D inspection and many others. 
> 
> Our ultimate goal is to continuously reduce the cost of industrial 3D cameras with the help of deeplearning's capabilities of abstract representation, and allow machine vision and AI to penetrate all aspects of this carbon-based world.
> 

  

| High Scan Quality  | Open Hardware | Easy-To-Start |
| ----- | ----- | ------- | 
| <div style="width: 150pt"> The camera is designed for industrial use. High accuracy of 50 micron or better is achievable </div> |  <div style="width: 150pt"> All of our developement is open-source, that anyone can use and improve it. </div>  | <div style="width: 150pt"> A growing number of detailed tutorials will help getting started. </div> |
| ![](images/products_sense_pic_pointcloud01@2x.png) | ![](images/products_sense_pic_pointcloud02@2x.png) | ![](images/products_sense_pic_pointcloud03@2x.png) |
| ![](images/products_sense_pic_pointcloud04@2x.png) | ![](images/products_sense_pic_pointcloud05@2x.png) | ![](images/products_sense_pic_pointcloud06@2x.png) |

 <table>
    <tr>
        <th>Parameter</th>
        <th>Value</th>
    </tr>
    <tr>
        <td>Calibration Accuracy</td>
        <td>0.05mm</td>
    </tr>
    <tr>
        <td>Point Cloud Resolusion</td>
        <td>1920x1200</td>
    </tr>
    <tr>
        <td>Image Resolusion</td>
        <td>1920x1200</td>
    </tr>
    <tr>
        <td>Frame Rate</td>
        <td>1fps</td>
    </tr>
    <tr>
        <td>Baseline</td>
        <td>150mm</td>
    </tr>
    <tr>
        <td>Working Distance</td>
        <td>400 ~ 1000mm</td>
    </tr>
    <tr>
        <td>Data Interface</td>
        <td>Ethernet</td>
    </tr>
    <tr>
        <td>HFOV</td>
        <td>40°</td>
    </tr>
    <tr>
        <td>VFOV</td>
        <td>23°</td>
    </tr>
    <tr>
        <td>Dimensions</td>
        <td>207 x 127.5 x 50.5 mm</td>
    </tr>
    <tr>
        <td>Weight</td>
        <td>1230g</td>
    </tr>
    <tr>
        <td>CPU</td>
        <td>Quad-core Arm A57 processor @ 1.43 GHz</td>
    </tr>
    <tr>
        <td>GPU</td>
        <td>128-core Maxwell GPU</td>
    </tr>
    <tr>
        <td>Memory</td>
        <td>4GB 64-bit LPDDR4</td>
    </tr>
</table>

[更多型号参数 More camera models](https://github.com/Open3DV/Xema/wiki/DexSense)<br>
[物料清单 Bill of Materials](https://github.com/Open3DV/Xema/wiki/BOM.csv)<br>
[XEMA系列相机接口文档](https://github.com/Open3DV/Xema/wiki/SDK) [Interface Document of XEMA](https://github.com/Open3DV/Xema/wiki/Interface-Document-of-XEMA)<br>
[XEMA系列相机搭建指南](https://github.com/Open3DV/Xema/wiki/guidebook)<br>

<img src='images/modules.jpg' width=800>

## 结构件设计文件 Structural Design Files
<img src='https://github.com/Open3DV/xema_hardware/blob/main/image/XEMA-S assembly.png' width=300><br> 
<a href='https://github.com/Open3DV/xema_hardware/blob/main/step/XEMA-S%20assembly.stp'>XEMA-S scanning range (400mm-1000mm) step file </a> <a href='https://evercraft.co/share?key=502244'> (view online) </a>
<br> <br> 
<img src='https://github.com/Open3DV/xema_hardware/blob/main/image/XEMA-L assembly.png' width=300><br>
<a href='https://github.com/Open3DV/xema_hardware/blob/main/step/XEMA-L%20assembly.stp'>XEMA-L scanning range (1000mm-3000mm)step file </a><a href='https://evercraft.co/share?key=DE2860'> (view online) </a>
<br> <br> 
<img src='https://github.com/Open3DV/xema_hardware/blob/main/image/XEMA-S structural components.png' width=300><br>
<a href='https://github.com/Open3DV/xema_hardware/blob/main/step/XEMA-S%20structural%20components.stp'> XEMA-S structural components step file </a>
<a href='https://evercraft.co/share?key=92E956'> (view online) </a>
<br> <br>
<img src='https://github.com/Open3DV/xema_hardware/blob/main/image/XEMA-L structural components.png' width=300><br>
<a href='https://github.com/Open3DV/xema_hardware/blob/main/step/XEMA-L%20structural%20components.stp'> XEMA-L structural components step file </a>
<a href='https://evercraft.co/share?key=167319'> (view online) </a>
<br> 
## 电路设计文件 PCB Design Files(Pads format)
<a href='https://github.com/Open3DV/xema_hardware/blob/main/pcb/XEMA%20Jetson%20Nano%20Carrier%20Board.pcb'>XEMA Jetson Nano carrier board  layout (pcb)</a>
<br> <a href='https://github.com/Open3DV/xema_hardware/blob/main/pcb/XEMA%20Jetson%20Nano%20Carrier%20Board.sch'>XEMA Jetson Nano carrier board schematics (sch)</a>

## 淘宝购买链接 Purchase XEMA camera or its DIY kit in the Taobao links below:<br>
<a href='https://item.taobao.com/item.htm?spm=a230r.7195193.1997079397.8.3dc842deinrFRl&id=693155032629&abbucket=6'>XEMA-S / XEMA-L Camera </a><br>
<a href='https://item.taobao.com/item.htm?spm=2013.1.w4004-24671353614.6.243b11c2sjZSTQ&id=694002907610'>XEMA-S DIY kits</a><br>
<a href='https://item.taobao.com/item.htm?spm=a230r.7195193.1997079397.9.3dc842deinrFRl&id=693408027639&abbucket=6'>XEMA-L DIY kits</a><br>

## XEMA相机DIY视频教程 -- XEMA camera DIY video tutorial (youtube/youku)
[Video Tutorial](https://github.com/Open3DV/Xema/discussions/42)

## Contact:
<img src='images/logo/dexforce.png' width=150 ></img><br>
Wechat: W13631674014 <br>
Email: <wudi@dexforce.com>

## Partners:
 <table>
    <tr>
        <td ><div style="width: 150pt"><img src='images/logo/seeed.png' width=100> <br> <a href='https://www.seeedstudio.com/pcb-assembly.html'>Fusion PCB Assembly Service </a> </div></td>
        <td><div style="width: 150pt"><img src='images/logo/nvidia_inception_program.jpg' width=100></div></td>
        <td><div style="width: 100pt"><img src='images/logo/anhua.png' width=100></div></td>
        <td><div style="width: 100pt"><img src='images/logo/cbc.png' width=100></div></td>
    </tr>
    <tr>
        <td><img src='images/logo/daheng.png' width=100> </td>
        <td><img src='images/logo/basler.png' width=100></td>
        <td></td>
        <td></td>
    </tr>
</table>
