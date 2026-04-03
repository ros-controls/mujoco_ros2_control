# Tips for Modeling Complex Geometries

General tips for modeling complex geometries in MuJoCo.
These lessons learned are largely based on use cases explored within the NASA Johnson Space Center [iMETRO facility](https://github.com/NASA-JSC-Robotics/iMETRO).

## Object URDFs

It is helpful to create a URDF to combine an object's sub-parts and keep these object URDFs in an environment/mockups package (such as the [surface mockups description](https://github.com/NASA-JSC-Robotics/surface_robotics_mockups_description/tree/main) package used in the [CLR workspace](https://github.com/NASA-JSC-Robotics/clr_ws)).

Having the object URDFs will make it easier to convert these objects to MJCFs later, as described in the [MJCF conversion documentation](./TOOLS.md).

## STL File Type

STLs come in two types: ASCII and binary.
CAD can export both types of STL, but RViz can only read binary STL files.
ASCII STL files can be converted to binary using the Python command line tool [`stlconverter`](https://pypi.org/project/stlconverter/).

## Interactions with Convex Hulls in MuJoCo

Determining how to model complex geometries depends on how the robot (or anything in the simulation) will interact with the objects.
When the STL files exported from CAD are run through the [conversion process](./TOOLS.md), MuJoCo draws convex hulls for collision checking around the model.
If an object is modeled as a singular asset, the convex hull will prevent the robot from interacting with any meaningful features of the object.
Instead, objects should be modeled as separate sub-parts that are brought together in the [object URDF](#object-urdfs).

We can see an example of how a low-fidelity lunar habitat concept is modeled in MuJoCo.

> [!WARNING]
> This depiction of a lunar habitat concept does not constitute an official design or official endorsement, either expressed or implied, by NASA.

The low-fidelity lunar habitat concept in MuJoCo:
![Habitat Visual](./images/habitat.png "The low-fidelity lunar habitat concept")

An example of a convex hull around a low-fidelity lunar habitat concept modeled as a singular asset is depicted below; the robot is unable to reach onto the habitat porch because of how the convex hull is drawn around this asset.
![Poor Convex Hull](./images/habitat-convex-hull-bad.png "Convex hull around object modeled as singular asset")

Instead, models should be exported from CAD based on the meaningful sub-parts the robot will interact with.
The lunar habitat, for example, is exported as 3 sub-parts: the habitat module itself, the porch, and the stairs/ladder leading up to the porch.
The convex hull for the object with modeled sub-parts is depicted below.
![Convex Hull](./images/habitat-convex-hull.png "Convex hull around object modeled with sub-parts")

Note that with this object breakdown, the robot *cannot* reach into the module itself, since the convex hull around the model closes any openings into the module.
Be aware that how objects are broken into sub-parts will depend on the application and any expected interactions with the object.
