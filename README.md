# python_pcd

This package provides I/O code for working with PCL .pcd pointcloud files in 
pure python. For a Python wrapping of the C++ PCL functions see 
http://strawlab.github.io/python-pcl/

## Writing

Example:

```python
import python_pcd

message = sensor_msgs.msg.PointCloud2() # or a message from somewhere else
python_pcd.write_pcd("this.pcd", message)
```

the definition of `write_pcd`:

```python
def write_pcd(filename,  pointcloud, overwrite=False, viewpoint=None,
              mode='binary'):
    """
    Writes a sensor_msgs::PointCloud2 to a .pcd file.
    :param filename - the pcd file to write
    :param pointcloud - sensor_msgs::PointCloud2 to write to a file
    :param overwrite - if True, allow overwriting existing files
    :param viewpoint - the camera viewpoint, (x,y,z,qw,qx,qy,qz)
    :param mode - the writing mode: 'ascii' for human readable, 'binary' for
                  a straight dump of the binary data, 'binary_stripped'
                  to strip out data padding before writing (saves space but it slow)
    """

```
## Reading

Not implemented.