var canvasWidth, canvasHeight;

function initializeGL(canvas)
{
    canvasWidth  = canvas.width;
    canvasHeight = canvas.height;

    scene = new THREE.Scene();

    camera = new THREE.PerspectiveCamera( 30, canvas.width / canvas.height, 0.001, 1000 );
    camera.position.x = 4.0;
    camera.position.y = zeroVector.y;
    camera.lookAt(zeroVector);

    var axisHelper = new THREE.AxisHelper(2.5);
    scene.add(axisHelper);

    initOrbitControl();


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

function resizeGL(canvas)
{
    camera.aspect = canvas.width / canvas.height;
    camera.updateProjectionMatrix();

    renderer.setPixelRatio(canvas.devicePixelRatio);
    renderer.setSize( canvas.width, canvas.height );

    canvasWidth  = canvas.width;
    canvasHeight = canvas.height;
}

function degToRad(degrees)
{
    return degrees * Math.PI / 180;
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

    //Usage:
    /*
        //orbit about zeroVector
        var cameraPosition = moveCamera(canvas.xRot, canvas.yRot, canvas.distance);
        camera.position.x = cameraPosition[0];
        camera.position.y = cameraPosition[1];
        camera.position.z = cameraPosition[2];
    */
}
