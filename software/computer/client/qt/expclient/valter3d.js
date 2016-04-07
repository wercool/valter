Qt.include("qrc:/valter3d/json_models_loader.js")
Qt.include("qrc:/valter3d/resources/3rdparty/three.js")

var camera, scene, renderer;
var light1, light2, light3
var zeroVector = new THREE.Vector3(0, 1.2, 0);

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

var valterGroup = new THREE.Object3D();

function initializeGL(canvas)
{    
    scene = new THREE.Scene();

    camera = new THREE.PerspectiveCamera( 30, canvas.width / canvas.height, 0.001, 1000 );
    camera.position.x = 4.0;
    camera.position.y = zeroVector.y;
    camera.lookAt(zeroVector);

    var axisHelper = new THREE.AxisHelper(2.5);
    scene.add(axisHelper);


    light1 = new THREE.DirectionalLight( 0xF5F9FA, 0.25 );
    light1.position.y = 3;
    light1.position.x = 3;
    scene.add( light1 );

    light2 = new THREE.HemisphereLight( 0xF5F9FA, 0xF5F9FA, 0.5 );
    light2.position.y = 3;
    scene.add( light2 );

    light3 = new THREE.DirectionalLight( 0xF5F9FA, 1.0 );
    light3.position.y = 3;
    light3.position.x = -3;
    scene.add( light3 );

    renderer = new THREE.Canvas3DRenderer({ canvas: canvas, antialias: true, devicePixelRatio: canvas.devicePixelRatio });
    renderer.setSize( canvas.width, canvas.height );
    renderer.setClearColor( 0xE6EFFF );
    renderer.setPixelRatio(canvas.devicePixelRatio);

    loadModels();
}

function paintGL(canvas)
{
    if (canvas.mousePrevX === 0)
        canvas.mousePrevX = canvas.mouseX;
    if (canvas.mousePrevY === 0)
        canvas.mousePrevY = canvas.mouseY;

    if (canvas.panCamera)
    {
        console.log("!!!!");
    }

    var cameraPosition = moveCamera(canvas.xRot, canvas.yRot, canvas.distance);
    camera.position.x = cameraPosition[0];
    camera.position.y = cameraPosition[1];
    camera.position.z = cameraPosition[2];

    camera.lookAt(zeroVector);

    canvas.mousePrevX = canvas.mouseX;
    canvas.mousePrevY = canvas.mouseY;


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

function resizeGL(canvas)
{
    camera.aspect = canvas.width / canvas.height;
    camera.updateProjectionMatrix();

    renderer.setPixelRatio(canvas.devicePixelRatio);
    renderer.setSize( canvas.width, canvas.height );
}

function mousePointerXYToSceneXZ(canvas)
{
    var pMouse = new THREE.Vector3((canvas.mouseX / canvas.width) * 2 - 1, -(canvas.mouseY / canvas.height) * 2 + 1, 0 );
    pMouse.unproject(camera);

    var m = pMouse.y / ( pMouse.y - camera.position.y );

    var pos = new THREE.Vector3(0, 0, 0);
    pos.x = pMouse.x + ( camera.position.x - pMouse.x ) * m;
    pos.z = pMouse.z + ( camera.position.z - pMouse.z ) * m;

    //console.log(pos.x, pos.z);
}

function moveCamera(xRot, yRot, distance)
{
    var xAngle = degToRad(xRot);
    var yAngle = degToRad(yRot);

    var zPos = distance * Math.cos(xAngle) * Math.cos(yAngle);
    var xPos = distance * Math.sin(xAngle) * Math.cos(yAngle);
    var yPos = distance * Math.sin(yAngle);

    return [-xPos, yPos, zPos];
}

function degToRad(degrees)
{
    return degrees * Math.PI / 180;
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
