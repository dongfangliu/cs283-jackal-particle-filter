# Report

## Algorithm

1. parse real tag pose from tag_map.txt
2. spawn a lot of pose samples uniformly in the area
3. while moving, motion_update updates the pose of the samples
4. while tags being detected,  we start from each sample and estimate the corresponding tag poses, then compare the pose with the real tag pose to update the weights
5. resample if necessary ( using select and replacement algorithm)

## Reference

http://cecas.clemson.edu/~ahoover/ece854/lecture-notes/lecture-pf.pdf

## Result

![1608108414860](C:\Users\liu\AppData\Roaming\Typora\typora-user-images\1608108414860.png)

![1608108432097](C:\Users\liu\AppData\Roaming\Typora\typora-user-images\1608108432097.png)

![1608108445558](C:\Users\liu\AppData\Roaming\Typora\typora-user-images\1608108445558.png)

![1608108458642](C:\Users\liu\AppData\Roaming\Typora\typora-user-images\1608108458642.png)