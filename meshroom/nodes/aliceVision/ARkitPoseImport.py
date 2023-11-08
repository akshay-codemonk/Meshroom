__version__ = "2.0"

from meshroom.core import desc
import os
import numpy as np
import json

class ARkitPoseImport(desc.AVCommandLineNode):
    commandLine = 'aliceVision_ARkitPoseImport {allParams}'
    size = desc.DynamicNodeSize('input')
    category = 'Other'
    documentation = '''
    Import arkit poses. The imported poses override calculated camera poses from SFM node.
    That script expects the frame intrinsic json to be in a directory called cameras aside the input rgb images folder, and have similar name (eg Image/0000234.jpg camera/0000234.json)
    '''

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
            label='Poses imported from ARkit',
            description='Output cameras file.',
            value=desc.Node.internalFolder + "/" + "cameras.sfm",
            uid=[],
        ),
    ]


    def processChunk(self, chunk):
        try:
            self._stopped = False
            chunk.logManager.start(chunk.node.verboseLevel.value)
            chunk.logger.info("started")

            inputCamerasFile = chunk.node.input.value
            outputCamerasFile = chunk.node.output.value

            self.importPoses(chunk, inputCamerasFile, outputCamerasFile)

            chunk.logger.info("ended")
        except Exception as e:
            chunk.logger.error(e)
            raise RuntimeError()
        finally:
            chunk.logManager.end()

    def stopProcess(self, chunk):
        self._stopped = True


    def importPoses(self, chunk, inputCamerasFile, outputCamerasFile):
        f = open(inputCamerasFile,)
        sfm_data = json.load(f)
        sfm_data["poses"] = []
        
        for view in sfm_data["views"]: 
            if self._stopped: raise RuntimeError("User asked to stop")
            image_path = view["path"]
            pose_folder_path = os.path.join(os.path.dirname(image_path), "../Camera")  # Go one directory back and choose poses folder
            image_file_name = image_path.split('/')[-1].split('.')[0]  + ".json"
            pose_file_path = os.path.join(pose_folder_path, image_file_name)
            if not os.path.isfile(pose_file_path): raise Exception("Pose file not found", pose_file_path, "check if the file exists")
            poseId = view['poseId']
            with open(pose_file_path, 'r') as pose_file:
                    camera_data = json.load(pose_file)
                    arkit_m = camera_data.get("transform_matrix",None)
                    if arkit_m is None:
                        arkit_m = [camera_data[f"t_{i}{j}"] if f"t_{i}{j}" in camera_data else 0 for i in range(3) for j in range(4)]
                        # Append the last row of the transformation matrix
                        arkit_m.extend([0, 0, 0, 1])
                    # Reshape the t_values into a 4x4 matrix
                    arkit_matrix = np.array(arkit_m).reshape(4, 4)
                    tmp_vec = np.array([1.0, -1.0, -1.0, 1.0])
                    coordinate_transform = np.diag(tmp_vec)
                    transform_matrix = np.dot(arkit_matrix, coordinate_transform)
                    rotation_matrix = transform_matrix[0:3,0:3]
                    camera_rotation = rotation_matrix.flatten().tolist()
                    camera_center = [camera_data[f"t_{i}3"] for i in range(3)]
                    pose_data = {"poseId": poseId, "pose" : {'transform':{'rotation' : camera_rotation, 'center' : camera_center},'locked':"1"}}
                    sfm_data["poses"].append(pose_data)
                    chunk.logger.info("loaded " + pose_file_path + " for image " + image_path)    
            with open(outputCamerasFile, 'w') as output_file:
                json.dump(sfm_data, output_file, indent=4)