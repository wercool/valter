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

var trunkGroup = new THREE.Object3D();
var bodyGroup = new THREE.Object3D();

var valterGroup = new THREE.Object3D();

function paintGL(canvas)
{
    switch (canvas.orbitControlState)
    {
        case 1:
            handleMouseMoveRotate(canvas.mouseX, canvas.mouseY);
        break;
        case 2:
            handleMouseMovePan(canvas.mouseX, canvas.mouseY);
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

            sceneInit = true;
        }
        else
        {
            //console.log(manEndEffectorGroup.localToWorld( manEndEffectorMesh.position.clone() ).y);
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
