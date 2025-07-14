
This pkg detects orange color in the images taken from camera sensor using OpenCV.  <br>
___

<br>
So we simply convert the raw images to HSV format. This is because BGR mixes brightness and color together, but in HSV, color (Hue) is separated from lighting (Value) — this makes it easier to detect a specific color under varying lighting.<br>
<br>  

We now define upper and lower ranges for masking the image. This range specifies what min and max values to consider for each Hue, saturation and value. The selected range in this pkg is for typical orange color.  <br>

Now we do masking. That is we just convert all the pixels lying in the above specified range to be white. and rest other pixels to be black.
<br>

Now we can simple count the number of orange pixels(i.e. white pixels) in the image.  
<br>

___