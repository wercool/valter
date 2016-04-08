Qt.include("qrc:/valter3d/resources/3rdparty/three.js")
Qt.include("qrc:/valter3d/utils.js")
Qt.include("qrc:/valter3d/json_models_loader.js")
Qt.include("qrc:/valter3d/orbitControls.js")

var zeroVector = new THREE.Vector3(0, 1.0, 0);

var camera, scene, renderer;
var light1, light2, light3;

var sceneInit = false;
var meshesLoaded = false;

var baseShiftY = 0.135; //wheel radius m

var platformGroup = new THREE.Object3D();
var frontLeftSupportWheelGroup = new THREE.Object3D();
var frontRightSupportWheelGroup = new THREE.Object3D();
var rearLeftSupportWheelGroup = new THREE.Object3D();
var rearRightSupportWheelGroup = new THREE.Object3D();
var leftWheelGroup = new THREE.Object3D();
var rightWheelGroup = new THREE.Object3D();

var manLink1Group = new THREE.Object3D();
var manLink2Group = new THREE.Object3D();
var manGripperTiltGroup = new THREE.Object3D();
var manGripperRotateGroup = new THREE.Object3D();
var manEndEffectorGroup = new THREE.Object3D();
var manEndEffectorMesh;
var prevManEndEffectorMeshPosition = new THREE.Vector3();

var trunkGroup = new THREE.Object3D();
var bodyGroup = new THREE.Object3D();
var leftShoulderGroup = new THREE.Object3D();
var rightShoulderGroup = new THREE.Object3D();
var leftShoulderLinkGroup = new THREE.Object3D();
var rightShoulderLinkGroup = new THREE.Object3D();
var leftArmLinkGroup = new THREE.Object3D();
var rightArmLinkGroup = new THREE.Object3D();
var leftElbowGroup = new THREE.Object3D();
var rightElbowGroup = new THREE.Object3D();
var leftForearmYawGroup = new THREE.Object3D();
var rightForearmYawGroup = new THREE.Object3D();
var leftForearmRollGroup = new THREE.Object3D();
var rightForearmRollGroup = new THREE.Object3D();
var leftPalmTiltGroup = new THREE.Object3D();
var rightPalmTiltGroup = new THREE.Object3D();
var leftPalmGroup = new THREE.Object3D();
var rightPalmGroup = new THREE.Object3D();

var leftFinger0Group = new THREE.Object3D();
var leftFinger1Group = new THREE.Object3D();
var leftFinger2Group = new THREE.Object3D();
var leftFinger3Group = new THREE.Object3D();
var leftFinger4Group = new THREE.Object3D();
var leftFinger5Group = new THREE.Object3D();

var rightFinger0Group = new THREE.Object3D();
var rightFinger1Group = new THREE.Object3D();
var rightFinger2Group = new THREE.Object3D();
var rightFinger3Group = new THREE.Object3D();
var rightFinger4Group = new THREE.Object3D();
var rightFinger5Group = new THREE.Object3D();

var headYawGroup = new THREE.Object3D();

var valterGroup = new THREE.Object3D();

function paintGL(canvas)
{
    switch (canvas.mouseButtonPressed)
    {
        case 1:
            mousePointerXYToSceneXZ(canvas);
        break;
        case 2:
            handleMouseMovePan(canvas.mouseX, canvas.mouseY);
        break;
        case 3:
            handleMouseMoveRotate(canvas.mouseX, canvas.mouseY);
        break;
    }

    if (meshesLoaded)
    {
        if (!sceneInit)
        {
            valterGroup.add(platformGroup);
            valterGroup.add(leftWheelGroup);
            valterGroup.add(rightWheelGroup);
            valterGroup.add(frontLeftSupportWheelGroup);
            valterGroup.add(frontRightSupportWheelGroup);
            valterGroup.add(rearLeftSupportWheelGroup);
            valterGroup.add(rearRightSupportWheelGroup);

            valterGroup.add(manLink1Group);

            scene.add(valterGroup);

            valterGroup.position.y += baseShiftY;

//            trunkGroup.rotation.y = 0.5;
//            bodyGroup.rotation.z = -0.5;
//            leftShoulderGroup.rotation.y = -0.5;
//            rightShoulderGroup.rotation.y = 0.5;
//            leftShoulderLinkGroup.rotation.z = 0.5;
//            rightShoulderLinkGroup.rotation.z = 0.5;
//            leftArmLinkGroup.rotation.x = 0.5;
//            rightArmLinkGroup.rotation.x = -0.5;
//            leftElbowGroup.rotation.z = 0.5;
//            rightElbowGroup.rotation.z = 0.5;
//            leftForearmYawGroup.rotation.y = -0.5;
//            rightForearmYawGroup.rotation.y = 0.5;
//            leftPalmTiltGroup.rotation.z = 0.5;
//            rightPalmTiltGroup.rotation.z = 0.5;
//            leftPalmGroup.rotation.y = -0.5;
//            rightPalmGroup.rotation.y = 0.5;

            sceneInit = true;
        }
        else
        {
            if (!prevManEndEffectorMeshPosition.equals ( manEndEffectorGroup.localToWorld(manEndEffectorMesh.position.clone()) ))
            {
                prevManEndEffectorMeshPosition = manEndEffectorGroup.localToWorld(manEndEffectorMesh.position.clone());
                drawPixel(prevManEndEffectorMeshPosition);
            }
        }
    }

    canvas.updateLabels();

    renderer.render( scene, camera );
}

function setValterGroupRotationY(angle)
{
    valterGroup.rotation.y = angle;
}

function setLink1ZAngle(angle)
{
    manLink1Group.rotation.z = angle;
}

function setLink2ZAngle(angle)
{
    manLink2Group.rotation.z = angle;
}

function setManTiltZAngle(angle)
{
    manGripperTiltGroup.rotation.z = angle;
}
