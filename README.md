# Gore Pipeline (Keypoint and Feature Extraction)
This program is the first part of the mesh registration pipeline implemented as a part of our project, The Global Alignment of Meshes for the Microsoft HoloLens. Its purpose is to transform a HoloLens mesh, inputted as an obj file, into a format that can be run through the GORE algorithm code provided by the orginal authors Bustos and Chin. The output of this program is three csv files containing a point cloud, keypoints, and features. These csv files can then be read into the original Matlab program in which GORE is implemented. 

### Setting user parameters
On the top of the main file under the section "User defined parameters", there are several values that need to be defined by the user. This includes the input file path, the output file name, the feature search radius, normal search radius, percentage of noise to generate, and amount of noise to generate. 
