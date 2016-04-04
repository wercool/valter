Qt.include("qrc:/valter3d/resources/3rdparty/three.js")

var camera, scene, renderer;
var cameraLight;
var zeroVector = new THREE.Vector3(0, 0, 0);

var valterBaseMesh1;
var valterBaseMesh2;
var valterBaseMeshReady = false;
var rotationSpeed = 0.0;

var group = new THREE.Object3D();

function initializeGL(canvas)
{
    scene = new THREE.Scene();

    camera = new THREE.PerspectiveCamera( 75, canvas.width / canvas.height, 0.001, 1000 );
    camera.position.y = 1;
    camera.position.x = 5;

    var light = new THREE.AmbientLight( 0x666666 );
    scene.add( light );

    cameraLight = new THREE.DirectionalLight( 0xffffff, 1 );
    cameraLight.position.y = camera.position.y;
    scene.add( cameraLight );

    renderer = new THREE.Canvas3DRenderer({ canvas: canvas, antialias: true, devicePixelRatio: canvas.devicePixelRatio });
    renderer.setSize( canvas.width, canvas.height );
    renderer.setClearColor( 0x000000 );

    var loader = new THREE.JSONLoader();

    loader.load( "qrc:/valter3d/valterBaseModel.json", function ( geometry, materials ) {
        geometry.computeVertexNormals();
        var bufferGeometry = new THREE.BufferGeometry();
        bufferGeometry.fromGeometry(geometry);
        var valterBaseMeshMaterialColor = 0xff0000;
        var valterBaseMeshMaterial = new THREE.MeshPhongMaterial({color: "#919191", specular: "#555555", shininess: 5});
        valterBaseMesh1 = new THREE.Mesh( bufferGeometry, valterBaseMeshMaterial );
        valterBaseMeshReady = true;
        valterBaseMesh1.position.y = 2;
        group.add(valterBaseMesh1);
    } );

    loader.load( "qrc:/valter3d/valterBaseModel.json", function ( geometry, materials ) {
        geometry.computeVertexNormals();
        var bufferGeometry = new THREE.BufferGeometry();
        bufferGeometry.fromGeometry(geometry);
        var valterBaseMeshMaterialColor = 0xff0000;
        var valterBaseMeshMaterial = new THREE.MeshPhongMaterial({color: "#919191", specular: "#555555", shininess: 5});
        valterBaseMesh2 = new THREE.Mesh( bufferGeometry, valterBaseMeshMaterial );
        valterBaseMeshReady = true;
        valterBaseMesh2.position.y = -2;
        group.add(valterBaseMesh2);
        scene.add( group );
    } );
}

function paintGL(canvas)
{
    camera.lookAt(zeroVector);

    if (valterBaseMeshReady)
    {
        valterBaseMesh1.rotation.y += rotationSpeed;
        group.rotation.y += 0.1;
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

function setSpeed(speed)
{
    rotationSpeed = speed;
}
