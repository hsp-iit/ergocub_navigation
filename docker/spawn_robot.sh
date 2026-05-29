#!/bin/bash
gz service -s /world/warehouse/create   --reqtype gz.msgs.EntityFactory   --reptype gz.msgs.Boolean   --req 'sdf_filename: "/usr/local/src/robot/robotology-superbuild/src/ergocub-software/urdf/ergoCub/robots/ergoCubGazeboV1_1_minContacts/model.urdf", name: "my_robot"'
