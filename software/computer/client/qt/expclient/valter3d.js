Qt.include("qrc:/valter3d/resources/3rdparty/three.js")

var camera, scene, renderer;
var light1, light2, light3
var zeroVector = new THREE.Vector3(0, 0, 0);
var baseShiftY = 0.135; //wheel radius m

var sceneInit = false;

var platformGroup = new THREE.Object3D();
var leftWheelGroup = new THREE.Object3D();
var rightWheelGroup = new THREE.Object3D();

var meshesLoaded = false;

var valterGroup = new THREE.Object3D();

var rotationSpeed = 0;

function initializeGL(canvas)
{
    scene = new THREE.Scene();

    camera = new THREE.PerspectiveCamera( 75, canvas.width / canvas.height, 0.001, 1000 );

    var axisHelper = new THREE.AxisHelper(1);
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

    var loader = new THREE.JSONLoader();

    //leftWheelChamberMesh
    loader.load( "qrc:/valter3d/resources/valter_model_json/wheel.chamber.json", function ( geometry, materials ) {
        geometry.computeVertexNormals();
        var bufferGeometry = new THREE.BufferGeometry();
        bufferGeometry.fromGeometry(geometry);
        var meshMaterial = new THREE.MeshPhongMaterial({color: "#1D1E1F", specular: "#555555", shininess: 1});
        var mesh = new THREE.Mesh( bufferGeometry, meshMaterial );
        mesh.position.z = -0.194;
        leftWheelGroup.add(mesh);
    } );

    //leftWheelMetalFrontMesh
    loader.load( "qrc:/valter3d/resources/valter_model_json/wheel.metal.front.json", function ( geometry, materials ) {
        geometry.computeVertexNormals();
        var bufferGeometry = new THREE.BufferGeometry();
        bufferGeometry.fromGeometry(geometry);
        var meshMaterial = new THREE.MeshPhongMaterial({color: "#E6E9EB", specular: "#555555", shininess: 0.5});
        var mesh = new THREE.Mesh( bufferGeometry, meshMaterial );
        mesh.position.z = -0.196;
        leftWheelGroup.add(mesh);
    } );

    //leftWheelMetalInternalMesh
    loader.load( "qrc:/valter3d/resources/valter_model_json/wheel.metal.internal.json", function ( geometry, materials ) {
        geometry.computeVertexNormals();
        var bufferGeometry = new THREE.BufferGeometry();
        bufferGeometry.fromGeometry(geometry);
        var meshMaterial = new THREE.MeshPhongMaterial({color: "#E6E9EB", specular: "#555555", shininess: 0.5});
        var mesh = new THREE.Mesh( bufferGeometry, meshMaterial );
        mesh.position.z = -0.191;
        mesh.rotation.y = degToRad(180);
        leftWheelGroup.add(mesh);
    } );

    //rightWheelChamberMesh
    loader.load( "qrc:/valter3d/resources/valter_model_json/wheel.chamber.json", function ( geometry, materials ) {
        geometry.computeVertexNormals();
        var bufferGeometry = new THREE.BufferGeometry();
        bufferGeometry.fromGeometry(geometry);
        var meshMaterial = new THREE.MeshPhongMaterial({color: "#1D1E1F", specular: "#555555", shininess: 1});
        var mesh = new THREE.Mesh( bufferGeometry, meshMaterial );
        mesh.position.z = 0.194;
        rightWheelGroup.add(mesh);
    } );

    //rightWheelMetalFrontMesh
    loader.load( "qrc:/valter3d/resources/valter_model_json/wheel.metal.front.json", function ( geometry, materials ) {
        geometry.computeVertexNormals();
        var bufferGeometry = new THREE.BufferGeometry();
        bufferGeometry.fromGeometry(geometry);
        var meshMaterial = new THREE.MeshPhongMaterial({color: "#E6E9EB", specular: "#555555", shininess: 0.5});
        var mesh = new THREE.Mesh( bufferGeometry, meshMaterial );
        mesh.rotation.y = degToRad(180);
        mesh.position.z = 0.196;
        rightWheelGroup.add(mesh);
    } );

    //rightWheelMetalInternalMesh
    loader.load( "qrc:/valter3d/resources/valter_model_json/wheel.metal.internal.json", function ( geometry, materials ) {
        geometry.computeVertexNormals();
        var bufferGeometry = new THREE.BufferGeometry();
        bufferGeometry.fromGeometry(geometry);
        var meshMaterial = new THREE.MeshPhongMaterial({color: "#E6E9EB", specular: "#555555", shininess: 0.5});
        var mesh = new THREE.Mesh( bufferGeometry, meshMaterial );
        mesh.position.z = 0.191;
        rightWheelGroup.add(mesh);
    } );

    //wheelAxisMesh
    loader.load( "qrc:/valter3d/resources/valter_model_json/wheel.axis.json", function ( geometry, materials ) {
        geometry.computeVertexNormals();
        var bufferGeometry = new THREE.BufferGeometry();
        bufferGeometry.fromGeometry(geometry);
        var meshMaterial = new THREE.MeshPhongMaterial({color: "#9EA2A3", specular: "#555555", shininess: 0.5});
        var mesh = new THREE.Mesh( bufferGeometry, meshMaterial );
        platformGroup.add(mesh);
    } );

    //wheelAxisFramesMesh
    loader.load( "qrc:/valter3d/resources/valter_model_json/wheel.axis.frames.json", function ( geometry, materials ) {
        geometry.computeVertexNormals();
        var bufferGeometry = new THREE.BufferGeometry();
        bufferGeometry.fromGeometry(geometry);
        var meshMaterial = new THREE.MeshPhongMaterial({color: "#072904", specular: "#555555", shininess: 1.5});
        var mesh = new THREE.Mesh( bufferGeometry, meshMaterial );
        platformGroup.add(mesh);
    } );

    //platformCaseMesh
    loader.load( "qrc:/valter3d/resources/valter_model_json/platfrom.case.json", function ( geometry, materials ) {
        geometry.computeVertexNormals();
        var bufferGeometry = new THREE.BufferGeometry();
        bufferGeometry.fromGeometry(geometry);
        var texture = THREE.ImageUtils.loadTexture("qrc:/resources/valter_model_json/textures/aluminum.jpg");
        texture.minFilter = THREE.NearestFilter;
        var meshMaterial = new THREE.MeshPhongMaterial({
                                                         map: texture,
                                                         bumpMap: texture,
                                                         bumpScale: 0.001,
                                                         shininess: 1.5
                                                     });
        meshMaterial.shading = THREE.FlatShading;
        var mesh = new THREE.Mesh( bufferGeometry, meshMaterial );
        mesh.position.y = 0.0345;
        platformGroup.add(mesh);


        meshesLoaded = true;
    } );
}

function paintGL(canvas)
{
    var eyePosition = moveEye(canvas.xRot, canvas.yRot, canvas.distance);
    camera.position.x = eyePosition[0];
    camera.position.y = eyePosition[1];
    camera.position.z = eyePosition[2];

    camera.lookAt(zeroVector);

    if (meshesLoaded)
    {
        if (!sceneInit)
        {
            valterGroup.add(platformGroup);
            valterGroup.add(leftWheelGroup);
            valterGroup.add(rightWheelGroup);
            scene.add(valterGroup);

            valterGroup.position.y += baseShiftY;

            sceneInit = true;
        }

        valterGroup.rotation.y += rotationSpeed;
    }

    renderer.render( scene, camera );
}

function resizeGL(canvas)
{
    camera.aspect = canvas.width / canvas.height;
    camera.updateProjectionMatrix();

    renderer.setPixelRatio(canvas.devicePixelRatio);
    renderer.setSize( canvas.width, canvas.height );
}

function moveEye(xRot, yRot, distance)
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

function setSpeed(speed)
{
    rotationSpeed = speed / 10;
}

function setGroupRotationX(angle)
{
    valterGroup.rotation.x = angle;
}
