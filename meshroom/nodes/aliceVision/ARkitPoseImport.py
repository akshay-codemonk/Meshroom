__version__ = "2.0"

from meshroom.core import desc
import os
import numpy as np
import json

class ARkitPoseImport(desc.AVCommandLineNode):
    # Command line definition
    commandLine = 'aliceVision_ARkitPoseImport {allParams}'
    size = desc.DynamicNodeSize('input')
    category = 'Other'
    documentation = '''
    Import ARKit poses. The imported poses override calculated camera poses from the SFM node.
    This script expects the frame intrinsic JSON to be in a directory called "cameras" alongside the input RGB images folder,
    and have similar names (e.g., Image/0000234.jpg, Camera/0000234.json).
    '''

    # Input and output definitions
    inputs = [
        desc.File(
            name="input",
            label="SfMData",
            description="Input cameras file from CameraInit.",
            value="",
            uid=[0],
        ),
        desc.ChoiceParam(
            name="verboseLevel",
            label="Verbose Level",
            description="Verbosity level (fatal, error, warning, info, debug, trace).",
            value="info",
            values=["fatal", "error", "warning", "info", "debug", "trace"],
            exclusive=True,
            uid=[],
        )
    ]

    outputs = [
        desc.File(
            name='output',
            label='Poses imported from ARKit',
            description='Output cameras file.',
            value=desc.Node.internalFolder + "/" + "cameras.sfm",
            uid=[],
        ),
    ]

    def processChunk(self, chunk):
        try:
            self._stopped = False
            chunk.logManager.start(chunk.node.verboseLevel.value)
            chunk.logger.info("Started")

            inputCamerasFile = chunk.node.input.value
            outputCamerasFile = chunk.node.output.value

            self.importPoses(chunk, inputCamerasFile, outputCamerasFile)

            chunk.logger.info("Ended")
        except Exception as e:
            chunk.logger.error(e)
            raise RuntimeError()
        finally:
            chunk.logManager.end()

    def stopProcess(self, chunk):
        self._stopped = True

    def importPoses(self, chunk, inputCamerasFile, outputCamerasFile):
        # Open input cameras file
        with open(inputCamerasFile) as f:
            sfm_data = json.load(f)
            sfm_data["poses"] = []

        # Iterate over views in the SFM data
        for view in sfm_data["views"]:
            if self._stopped:
                raise RuntimeError("User asked to stop")

            # Extract image and pose paths
            image_path = view["path"]
            pose_folder_path = os.path.join(os.path.dirname(image_path), "../Camera")
            image_file_name = image_path.split('/')[-1].split('.')[0] + ".json"
            pose_file_path = os.path.join(pose_folder_path, image_file_name)

            # Check if pose file exists
            if not os.path.isfile(pose_file_path):
                raise Exception("Pose file not found", pose_file_path, "Check if the file exists")

            # Extract pose data from ARKit JSON
            poseId = view['poseId']
            with open(pose_file_path, 'r') as pose_file:
                camera_data = json.load(pose_file)
                arkit_m = camera_data.get("transform_matrix", None)

                # Handle missing transform matrix
                if arkit_m is None:
                    arkit_m = [camera_data[f"t_{i}{j}"] if f"t_{i}{j}" in camera_data else 0 for i in range(3) for j in range(4)]
                    arkit_m.extend([0, 0, 0, 1])  # Append the last row of the transformation matrix

                # Reshape the t_values into a 4x4 matrix
                arkit_matrix = np.array(arkit_m).reshape(4, 4)
                tmp_vec = np.array([1.0, -1.0, -1.0, 1.0])
                coordinate_transform = np.diag(tmp_vec)
                transform_matrix = np.dot(arkit_matrix, coordinate_transform)
                rotation_matrix = transform_matrix[0:3, 0:3]

                # Extract rotation matrix and camera center
                camera_rotation = [str(element) for element in rotation_matrix.flatten()]
                camera_center = [camera_data[f"t_{i}3"] for i in range(3)]
                camera_center = [str(element) for element in camera_center]

                # Create and append pose data
                pose_data = {"poseId": poseId, "pose": {'transform': {'rotation': camera_rotation, 'center': camera_center}, 'locked': "1"}}
                sfm_data["poses"].append(pose_data)
                chunk.logger.info("Loaded " + pose_file_path + " for image " + image_path)

        # Write the modified SFM data to the output file
        with open(outputCamerasFile, 'w') as output_file:
            json.dump(sfm_data, output_file, indent=4)
