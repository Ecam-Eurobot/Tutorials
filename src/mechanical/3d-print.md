# 1. How to think 3D printing
Now that you know how to design in 3D and export your file to a slicer, we'll take you straight to the world of the 3D printing. We'll see his amazing abilities but also his limitations.

## 1.1. Think "mechanical properties"
When you'll have to create a mechanical piece for your robot, you have to think about the way you'll use it.
For example, if it's a piece that will be in contact with water, a lot of people will have the tendency to say: "You have to use ABS!"
Right now there are other possibilities as the PETG that have similar properties but has less warping. Each one of them have their advantages and their defaults.
It's true that the ABS will have better mechanical resistance against wear and tear than PLA but you'll have to pay more attention at the temperature variation and the wrapping problems.
 That's only the top of the iceberg for the materials, if you're interested by knowing more about it, you can still find a lot on the web on divers site like: [Primant3D](http://www.primante3d.com/materiaux/).

Another simple tip is to think about the way the fibers during the use of the pieces.
As you can see on this piece, a Pelton wheel , at first the shaft and the wheel were design as in one piece.
![img](img/mechanical/3d-print/Pelton_wheel_1.PNG)

After thought, as the shaft will have to oppose strength in the radial way, it's way more efficient to print it horizontaly than verticaly (where the layers would detache).
For the wheel, it's the opposite. The spoons have to go against strength in the perpendicular way, so it had to be printed also horizontal, laying on the bed.
It was then decided to print them in two pieces.
![img](img/mechanical/3d-print/Pelton_wheel_2.PNG)

## 1.2. Think about the supports
As you'll see in the point **3. Printer Configuration**, there are a lot of different parameters to take into account. In this point, we'll speak about the supports, their necessity and their troubles.
As you can see on the picture below, there a lots of support placed to help the printer. As the print is done layer by layer, it needs supports to help the extrusion of part on empty space.
![img](img/mechanical/3d-print/Support_1.PNG)
*Note that you'll have to be in layer view to see them (Top-Right Corner)*

The most used to tune your print is the **Overhang Angle**, it will allow you to control the amount and place of the supports.
![img](img/mechanical/3d-print/Support_2.PNG)

The use of limiting the amount of support is to reduce waste, reduce printing time (in this case 1h45) but also cleaning time as you'll have to cut or tear them apart by hand. There is a possibility with dual extrusion to print them with PVA (water soluble filament) but it requires a more expensive printer.

However, I recommend to try the parameters shown on the picture because they make the removing of supports way easier than the default one.

## 1.3. Think about the face laying on the heat bed
As said before, the printer will often need support, but the orientation of the piece on the bed will also help a lot to reduce them.
In this piece, simply by laying it on one face, you'll remove all the need for support.
![img](img/mechanical/3d-print/Bed_1.PNG)
![img](img/mechanical/3d-print/Bed_2.PNG)

In this case, it would be absurd to put it on the other face as it will require a lot of support and their will be very complex to remove.
![img](img/mechanical/3d-print/Bed_3.PNG)

On this final picture, imagine many of this cubes needing to be plugged one in another and that have to be printed on their left face.
You can use two different design:
- In the first design (the two top cube) the top-left one will require support.
- In the second design (the two cube below) the low-left one can be printed easily.

More, in the second design, all the cube are the same so you can print a lot of them really fast.
![img](img/mechanical/3d-print/Bed_4.PNG)

# 2. What printer to choose?
The offer in mater of 3D printers is enormous and can be confusing to an unexperimented printer.
And even if you know what you are looking for, you can find the same looking printer for at least 10 different prices and brands.
As a matter of fact a lot of companies copy the "big ones" and sale these copies a lot cheaper.
These copies can be a very good investment for non-professional work and can save you quite a few euros, but can also be a real nightmare.

A general advice is to read as many reviews as possible and to try to choses a printer that a lot of people have. Isolated brands are often isolated for a very good reason (quality issues for example) and you'll struggle finding informations on settings, performances and things like parts to print to upgrade you printer.

The Anet company for example is very popular brand that sells versions of open source printers.
The are close to the cheapest you can find, but have very decent quality and high liability.
Hereby lots of people have one and you can find tones of parts on
[Thingiverse AnetA8](https://www.thingiverse.com/search/page:1?q=Anet+A8&sa=&dwh=525af4b8e015a6f) to upgrade their models.

Other companies like FLSUN sale some of the same open source models (prusa i3) a like more expensive but are less reliable and upgrade parts are pretty difficult to find.

So what to chose ?

Well we are not going to give you a list of printers to buy or not to buy, but we'll try to give you guidelines to follow when comparing what the market offers.

## 2.1. Technologies
Men have developed different ways to 3D print stuff, each and every one of it with its own advantages.
The cheapest and thus most used is the extrusion of molten plastic, but others like resin printing (where a laser hits a bath of liquid resin hardening it instantly) or metal printing (same idea as for resin but with a bath of metal powder), are starting to emerge. These however are really expensive (2 000$ up to hundreds of thousands of dollars for the machine only) for the moment and won't be discussed any further.

The rest of this document will so only speak about plastic extrusion printing.

## 2.2. Structure of the printer
You'll find two main types of structures: cartesian and delta.
![img](img/mechanical/3d-print/Cartesian-3D-printer.jpg)

Delta 3D printers were designed for speed, but they also have the distinction to have a print bed that never moves, which may come in handy for certain print jobs.
In the other hand, their speed comes with a certain weakness for details and a lack in precision.

Cartesian printers are better for details and easier to build and maintain, but slower.

## 2.3. Precision
As you might expect, cheap printers (100€-400€) don't have the same accuracy as semi-professional ones that cost about 2000€-5000€ (Ultimakers for example).
This can be scaring at first, but think about it this way: do you really need to print at a resolution of 20 microns (an aluminum sheet is about 16 microns )?
For most of the prints you'll make a precision of 0,2mm and a little bit of sanding is more than enough.
You can find more than decent printers for around 200€ (Anet A6) for medium bed sizes.
Larger bed printers will cost about 300€ (Creality3D CR - 10).
Note that these printers are DIY printers so you'll have to assemble them your own.
This might take a few days, but is a piece of cake with the manual and will save you a few hundred euros.

## 2.4. Links
To close the subject, here are some links you could find handy in your search of a 3Dprinter to acquire or to improve.
- [Thingiverse AnetA8](https://www.thingiverse.com/search/page:1?q=Anet+A8&sa=&dwh=525af4b8e015a6f)
- [Instructables improvement](http://www.instructables.com/id/Transform-a-chinese-3D-printer-in-a-high-precision/)
- [Choose between a Bowden or Direct extruder](http://www.fabbaloo.com/blog/2015/11/11/bowden-or-direct-a-primer-on-extruder-styles)
- [Anet A8 vs A6](https://pevly.com/anet-a8-vs-a6/)
- [Anet A8 improvement](http://www.instructables.com/id/2000-Quality-Prints-From-a-200-Printer-an-Upgrade-/)
<!-- si tu veux rajouter des liens -->


# 3. Printer Configuration
There are tons of different parameters that will influence the quality of your print.
It would be out of the topic to list them all but we encourage you to read the following document to calibrate your printer as good as possible.

- [Cura's parameters for Anet A6](mechanical/References/A6_Parameters_Cura.pdf)
- [Print Quality Troobleshooting Guide Anet](mechanical/References/Print_Quality_Troobleshooting_Guide-Anet.pdf)
- [Warpping problems](http://www.primante3d.com/curling-29082016/)



--------------------------------------
## Contributors
- Puissant Baeyens Victor, 12098, [MisterTarock](https://github.com/MisterTarock)
- De Decker William, 14130, [WilliamHdd](https://github.com/WilliamHdd)
