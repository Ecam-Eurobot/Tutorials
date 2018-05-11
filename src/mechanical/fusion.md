# 1. Getting in touch with Fusion360
Fusion360 is a 3D modeling software used by many professionals and hobbyists to create pieces. You can directly send your models to slicers in order to print them or export the 2D plans to build it your own. The following chapters should give you a global overview of how Fusion360 works and what you can use it for. We'll try to orient our writing to learn your to design in order to print your models later and will end with some tips about the printing them.

To complete your learning we strongly recommend you subscribe to
[Lars Christensen's Youtube channel](https://www.youtube.com/channel/UCo29kn3d9ziFUZGZ50VKvWA).
He's a master in the use of Fusion360 and has very well made videos about every problem or question you might have.

3 videos in particular should hold your attention to get started:
[Fusion 360 Tutorial for Absolute Beginners— Part 1](https://www.youtube.com/watch?v=A5bc9c3S12g) and of course
[Fusion 360 Tutorial for Absolute Beginners— Part 2](https://www.youtube.com/watch?v=HXRMzJWo0-Q) and
[Fusion 360 Tutorial for Absolute Beginners— Part 3](https://www.youtube.com/watch?v=zS8dYA_Iluc&t=614s).  

## Fusion360 environment
The first window you get when opening Fusion360 should be this.


 By clicking on ![img](img/mechanical/fusion/square.png) you access to the repository holding all your folders.
 A good practice is to sort them into other repositories according to their relevance. A little bit like you would sort your computer desktop.
 ![FOLDERS](img/mechanical/fusion/FOLDERS.png)

## Project view

![img](img/mechanical/fusion/Fusion1.png)

We'll let you listen to M.Christensen by clicking on the links to the videos we gave earlier to learn the basics of Fusion360. He's an expert on the subject and honestly the only good way to learn this kind of software is by watching someone do and copy. Please don't learn the shortcuts by heart, you'll learn the most used ones by practice.

# 2. Main Functions and Shotcuts
You should already be able to do your first model now, but in case you misted some of the many things explained by Lars, we would like to give you a quick recap.

To start, note that all functions are easily available trough the "s" key shortcut. Just type a word related to what you are searching for and  you'll probably find it directly. *Note that you can save them by clicking on the curved arrow.*

## 2.1. Shortcuts
Please don't study them, by the way you probably know most of them after  your first hour of practice . If it's still not the case, you'll find them here:
- S=Model Toolbox


- L=Line
- C=Circle
- X=Construction
- D=Dimension


- Q=Push/Pull
- M=Move
- J=Joint
- T=Trim (delete lines)


- Middel button of your mouse = Pan
- Maj+ middel button  = 3D move

## 2.2. Useful functions

### Offset
Offset is a very handy tool when it comes to drawing parallel lines.
![img](img/mechanical/fusion/Offset1.png)

The tools works only in sketch mode and you have to already have a reference line (or a shape).
Type the "o" key to open the "offset menu", you then have to select the reference line you want.
You can either drag and drop the cursor with your mouse or type the value you wish the offset to be.
![img](img/mechanical/fusion/Offset_drag.png)
![img](img/mechanical/fusion/OFFSET_VALUE.png)

You'll end with the line/shape you selected offset as you wanted.
![img](img/mechanical/fusion/Offset_finish.png)

### Sweep
Another useful tool to make pipe, slide and so on is the "sweep" function. Start by drawing a sketch on a
face. In our example it's going to be a double circle.
![img](img/mechanical/fusion/Sweep_1.PNG)

Then close your current sketch. Go to another face. Start a new sketch . Use "spine" for example and draw a line following as much point as you want.
![img](img/mechanical/fusion/Sweep_2.PNG)

You'll be able now to use "Sweep", select the space between the two circle as "profile", select the line as "path" and you'll automatically make a curved pipe!
![img](img/mechanical/fusion/Sweep_3.PNG)

We used it for example for the [slide](https://a360.co/2Jzo4oP) in the 2018's ball mechanism.

### Revolve
Revolve allows you to create objects as if you turned them on a lathe. It comes in handy to make wheels, cannons, ...
To used it you have to first create a shape around an axis. When you create that shape, you have to imagine it round. So you only have to draw "a half cut of your future object".
![img](img/mechanical/fusion/Vase_shape.png)

Type "s" to access the "search menu" and enter "revolve" to find the tool. You'll have to select the shape and the axis you want and press "enter".
![img](img/mechanical/fusion/vase_glass.png)

### Appearance
To make design easier for others to imagine or simply to choose colors and material purely esthetically, the "Appearance" tool is a must! You can archive more or less the same result with the "Physical Material" tool, but this is more often used for simulation. You can also combine those tools to get for example a steel pipe covered in leather without having to create the wrapping in a new component.
It is also very handy when creating complex assemblies with little pieces because the different appearances help to see those different pieces.

Appearance is really easy to use. Type "a", choose the look you like and drag-drop it on the body you want. You can directly drop it in the project tree or on the visual objet.  
![img](img/mechanical/fusion/Appearance_tool.png)

### Pattern
Real time savior, "Pattern" allows you to duplicate sketches, bodies and components on a given distance.
Multiple pattern tools exists. To see then go into the "search menu" and type "Pattern".
![img](img/mechanical/fusion/multiple_patterns.png)

As you can see in the above picture, you can create a circular, rectangular or "free style" pattern.
For both rectangular and circular patterns the ones with white square create patterns in sketches. The grey ones create patterns of volumes (bodies, components). We'll only present the sketch pattern, but the volumes work in a similar way.

#### Rectangular pattern
![img](img/mechanical/fusion/Pattern_open.png)

To apply rectangular patterns, select the desired shape and the amount of copies you want in the 2 directions. You can then enter (or drag-drop) the distance on which you want to copy that shape for both directions and click "ok".
![img](img/mechanical/fusion/Rect_patt_1side.png)
![img](img/mechanical/fusion/Rect_patt_2side.png)
![img](img/mechanical/fusion/Rect_pattern_done.png)

#### Circular pattern
Circular patterns work in a similar way accept the fact you have to choose a center point and you can choose to make a complete revolution around it, or on a given angle.
![img](img/mechanical/fusion/circ_patt-full.png)
![img](img/mechanical/fusion/circ_patt_angle.png)

*Note that it's also possible to draw a path and then follow it with our pattern.*

### Mirror
Available in sketch and also in 3D, the "Mirror" tool is very useful to create quickly twice the same things symmetrically from a line or a plane.

For the 3D tools, start by drawing your piece, in this case a notch to close a wall.
![img](img/mechanical/fusion/Mirror_1.PNG)

Use the Mirror tool, select the faces that you want to copy and then the plane.
![img](img/mechanical/fusion/Mirror_2.PNG)
*Note that if you select only the top face of your notch you won't be able to copy it because the program doesn't allow you to create a new face in the empty space. You have to select all the three faces of your notch.*

![img](img/mechanical/fusion/Mirror_3.PNG)

As we said, it's also available in sketch mode. So if you want to make two holes symmetrically. Just draw a circle. Draw a construction line that we'll use as our center line. Chose the distance from the center line, then select "Mirror". Select the circle as "Object" and in the "Mirror line" select the construction line
![img](img/mechanical/fusion/Mirror_4.PNG)
![img](img/mechanical/fusion/Mirror_5.PNG)

We can now make our two holes. Select the two of them (by holding  "ctrl" pressed down) and then cut them through the bodie.
![img](img/mechanical/fusion/Mirror_6.PNG)

### Fillet
As you used it in Autocad or similars, fillet allows you to joint two lines to make a curve.
In Fusion 360, as a 3D software, you'll also find the possibility to "curve" your 3D body.
![img](img/mechanical/fusion/Fillet_1.PNG)
<figure>
  <figcaption>Fig. - First Fillet is for sketch and the second one for 3D model.</figcaption>
</figure>

To use the sketch fillet, just select it and then click on the two lines to join.
![img](img/mechanical/fusion/Fillet_2.PNG)

In the 3D model, just select the edge and with the arrow or value chose the radius of your fillet.
![img](img/mechanical/fusion/Fillet_3.PNG)
![img](img/mechanical/fusion/Fillet_4.PNG)

### Chamfer
In the same way of thinking than for fillet, you'll have the possibility to make a chamfer.
![img](img/mechanical/fusion/Chamfer_1.PNG)
Select the tools and click on the edge.
![img](img/mechanical/fusion/Chamfer_2.PNG)
You'll also be able to do it with a curved edge (the one made with fillet)
![img](img/mechanical/fusion/Chamfer_3.PNG)
![img](img/mechanical/fusion/Chamfer_4.PNG)
*Note that in this case you'll be limited by the angle of the previous made fillet.*

Another option is to change the "Chamfer type" and select "Distance and angle" to make a chamfer with a selected angle.
![img](img/mechanical/fusion/Chamfer_5.PNG)


### Join
A joint is a link between two pieces that describes the way they move one on the other. It comes in handy when describing actions to external people, or when modeling pieces in an assembly to avoid collisions due to design mistakes.
To create joints, you have to first **ground components**. By grounding components you avoid them to move, and so you can use them as a static reference. Take *Cortex* for example, the base has been grounded and all the rest around him has been jointed.
![img](img/mechanical/fusion/cortex.png)

This means you can not drag it around. The second thing you have to do is **create rigid groups**. By searching for "rigid group" when hitting the "s" key you will enter the "rigid group" menu. You can now select all the component that should move together. You'll still be able to drag them around, but only as one group and not separately anymore. This is for example the case for the mecanum wheels as you can not move their different pieces separately.
After this is done, we can now **join modules together** ('j' key). To do so select the points of the modules that will touch after jointing starting with the one you want to be able to move.
![img](img/mechanical/fusion/joint_face1.png)
![img](img/mechanical/fusion/joint_face2.png)

After this is done you can select the type of joint you want and the axis along which to movement will have to be.
![img](img/mechanical/fusion/joint_type.png)
![img](img/mechanical/fusion/joint_defined.png)

After confirming the settings, you can right-click on the joint to edit the joints limits and inverse the natural movement.
![img](img/mechanical/fusion/joint_limit.png)

 To learn more about jointing component click on the link to Lars's video about that topic
 [Fusion 360 Tutorial — How to get a handle on Assembly and Joints in Fusion](https://www.youtube.com/watch?v=KQNgIfjMr84).

# 3. 3D Design export to plan or 3D Slicer  

## 3.1. Design export as drawing
One of the main reason you have to make a 3D model before starting the actual build of your project is the ease you get to make 2D plan you can send to a manufacture or use to build the pieces your own. Making this plans will save you a lot of time and material because all the testing on designs is made virtually and not with physical materials you have to buy, cut, test, re-cut , etc... To extract the plans of the design you made, right-click on the component you want and select "create drawing".
![img](img/mechanical/fusion/drawing_start.png)

A window pops up to specify the parts you want to include in the drawing and the format of the destination sheet (**be careful on that**) select the piece and click "ok".
![img](img/mechanical/fusion/drawing_drop.png)

A widget is automatically created and the component is now attached to your mouse. Notice that in the small window that comes with the widget you can select the view and scale you want to use. After dropping the component and clicking "ok" on the small window your drawing is fully generated and you can start editing it.
![img](img/mechanical/fusion/drawing_dimension.png)

By hitting the "d" key you can start indicating the dimensions you want to specify. Save to PDF and your done.

## 3.2. Export 3D design  (3D printing)
Now comes the fun part: 3D printing your own designs. You'll see that it is really satisfying to see something you designed your own "come to live" in the printer and to do so you will have to export you design as an '.stl' file.

First start by hitting the "make" icon in the top bar.
![img](img/mechanical/fusion/make_icon.png)

It will open a window asking you to select the component you want to print and where you want to send the stl file. As you might expect you have to click on the body or component you want to print (**One at the time!**). As most printers have an accuracy only as good as their nozzle diameter, an assembly will always be printed as one bloc! For that reason we recommend to only select bodies when printing an assembly. It takes more time to print, but if the pieces have to move their is no other way to avoid monolithic prints.


Now comes the moment you have to choose between sending your design to a facility (and generating an stl file) or printing it from your computer and send it to a slicer software (up studio for the small printer in ECAM's electronic lab, Ultimaker Cura, Repetier-host, etc...).
![img](img/mechanical/fusion/3Dprint_select_body.png)

To **create an stl file** unselect "Send to 3D print utility" and select ok.

To **3D print it from your computer** select "Send to 3D print utility" and link your slicer software by clicking on the folder icon. When the slicer is linked click "ok" and launch the print form your slicer that will have been open and loaded with the design by Fusion360.

# 4. Design import and modification
As you probably know there are tons of CAD drawings already made that can often help you in your designs.
You'll find them on different formats but in general, all of them can be open with fusion or a slicer to print them.

The best known databases are:
- [GrabCAD](https://grabcad.com/library)
- [Thingiverse](https://www.thingiverse.com/)

In [GrabCAD](https://grabcad.com/library), you'll find *.stp*, *.sta*, *.SLDPRT* files that you'll have to send into fusion to modify them and then send them in your slicer. You'll find really useful and complex design to add to yours to verify dimension and so on.
We used it for example for our [Home Automation Panel](https://a360.co/2wFFzCj) during the 2018 Edition, where the design of the [LCD](https://grabcad.com/library/lcd-screen-16x2-1), the [Arduino](https://grabcad.com/library/arduino-uno-18) and the [motor](https://grabcad.com/library/motor-gear-box-4) really helped us to fix the dimensions of the panel.

When you'll have the file that you want to implement in Fusion, just click on the upload button (top-left corner, on the project panel).

*Always use this method instead of opening the file with fusion directly or it will make errors during the conversion*
![img](img/mechanical/fusion/Upload_1.PNG)
![img](img/mechanical/fusion/Upload_2.PNG)

You now have the possibility to link these new designs into yours. Just open the main design to see it, right-click on the secondary design and "Insert Into Current Design". The now linked component will be shown with some chain link on it.
![img](img/mechanical/fusion/Upload_3.PNG)


In [Thingiverse](https://www.thingiverse.com/), you'll find *.stl* files, already thought for the 3D printing, with some tips on how to print them sometimes.

It's also possible to modify them with the [following steps](https://www.youtube.com/watch?v=-IE7kKxdTFY).


--------------------------------------
## Contributors
- Puissant Baeyens Victor, 12098, [MisterTarock](https://github.com/MisterTarock)
- De Decker William, 14130, [WilliamHdd](https://github.com/WilliamHdd)
