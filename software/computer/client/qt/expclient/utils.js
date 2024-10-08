//keys pressed
var ctrlPressed = false;

var canvasWidth, canvasHeight;

var pointCloud1Geometry;
var pointCloud1Material;
var pointCloud1;

function setMouse(canvas, mouseX, mouseY)
{
    mouse.x = ( mouseX / canvas.width ) * 2 - 1;
    mouse.y = - ( mouseY / canvas.height ) * 2 + 1;
}

function initializeGL(canvas)
{
    canvasWidth  = canvas.width;
    canvasHeight = canvas.height;

    scene = new THREE.Scene();

    camera = new THREE.PerspectiveCamera( 30, canvas.width / canvas.height, 0.001, 1000 );
    camera.position.x = 3.0;
    camera.position.z = 3.0;
    camera.position.y = zeroVector.y;
    camera.lookAt(zeroVector);

    var axisHelper = new THREE.AxisHelper(2.5);
    scene.add(axisHelper);

    initOrbitControl();


    light1 = new THREE.DirectionalLight( 0xF5F9FA, 0.5 );
    light1.position.y = 3;
    light1.position.x = 3;
    light1.position.z = 3;
    light1.castShadow = true;
    scene.add( light1 );

    light2 = new THREE.HemisphereLight( 0xE6EFFF, 0xE6EFFF, 0.5 );
    light2.position.y = 1.5;
    scene.add( light2 );

    light3 = new THREE.DirectionalLight( 0xF5F9FA, 0.5 );
    light3.position.y = 3;
    light3.position.x = -3;
    light3.position.z = -3;
    scene.add( light3 );

    renderer = new THREE.Canvas3DRenderer({ canvas: canvas, antialias: true, devicePixelRatio: canvas.devicePixelRatio });
    renderer.setSize( canvas.width, canvas.height );
    renderer.setClearColor( 0xE6EFFF );
    renderer.setPixelRatio(canvas.devicePixelRatio);

    pointCloud1Geometry = new THREE.Geometry();
    pointCloud1Geometry.vertices.push(new THREE.Vector3(0,2.5,0));
    pointCloud1Material = new THREE.PointCloudMaterial( { color: "#000000", size: 2, sizeAttenuation: false } );
    pointCloud1 = new THREE.PointCloud( pointCloud1Geometry, pointCloud1Material );

    scene.add(pointCloud1);

    loadModels();

    //mouse manipulation scene
    objectManipulationPlane = new THREE.Mesh(
        new THREE.PlaneBufferGeometry( 2000, 2000, 8, 8 ),
        new THREE.MeshBasicMaterial( { visible: false } )
    );
    scene.add( objectManipulationPlane );

    //helperSphere
    var bufferGeometry = new THREE.SphereGeometry( 0.03, 32, 32 );
    var meshMaterial = new THREE.MeshPhongMaterial({color: "#FF0000", specular: "#555555", shininess: 1.0});
    meshMaterial.transparent = true;
    meshMaterial.opacity = 0.2;
    helperSphere = new THREE.Mesh( bufferGeometry, meshMaterial );
    scene.add(helperSphere);
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

function addValterGroupHelpers()
{
    var meshMaterial = new THREE.MeshPhongMaterial({color: "#FFFF00", specular: "#555555", shininess: 1.0});
    var bufferGeometry = new THREE.SphereGeometry( 0.02, 32, 32 );
    valterGroupXLineHelperMesh = new THREE.Mesh();//new THREE.Mesh(bufferGeometry, meshMaterial);
    valterGroupXLineHelperMesh.position.x = 1.0;
    valterGroupXLineHelperMesh.position.y -= baseShiftY;
    valterGroup.add(valterGroupXLineHelperMesh);
}

function renderValterGroupHelpers()
{
    valterGroupXLineHelperMeshPosition = valterGroup.localToWorld(valterGroupXLineHelperMesh.position.clone());

    scene.remove(valterGroupXLine);

    var geometry = new THREE.Geometry();
    geometry.vertices.push( new THREE.Vector3(valterGroup.position.x, 0, valterGroup.position.z),  valterGroupXLineHelperMeshPosition);

    valterGroupXLine = new THREE.Line( geometry, headGroupHeadTargetLineMaterial );

    scene.add(valterGroupXLine);
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
    return pos;
}

function drawPixel(pos)
{
    pointCloud1Geometry.vertices.push(pos);
    var newGeometry = new THREE.Geometry();
    newGeometry.vertices = pointCloud1Geometry.vertices;
    scene.remove(pointCloud1);
    pointCloud1 = new THREE.PointCloud( newGeometry, pointCloud1Material );
    scene.add(pointCloud1);
    //console.log(pointCloud1Geometry.vertices.length);
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

var headGroupHeadTargetLineMaterial = new THREE.LineBasicMaterial({ color: 0x000000 });
var helperLineMaterial = new THREE.LineBasicMaterial({ color: 0xff00ff });

function drawHeadGroupHeadTargetLine()
{
    //console.log("HeadEEF: ", headEndEffectorPosition.x, headEndEffectorPosition.y, headEndEffectorPosition.z);
    //console.log("Target: ", headTarget.position.x, headTarget.position.y, headTarget.position.z);

    scene.remove(headGroupHeadTargetLine);

    var geometry = new THREE.Geometry();
    geometry.vertices.push( headEndEffectorPosition, headTarget.position );

    headGroupHeadTargetLine = new THREE.Line( geometry, headGroupHeadTargetLineMaterial );

    headGroupHeadTargetLine3 = new THREE.Line3(headEndEffectorPosition, headTarget.position);

    scene.add(headGroupHeadTargetLine);
}

function drawHelperLine1(start, end)
{
    scene.remove(helperLine1);

    var geometry = new THREE.Geometry();
    geometry.vertices.push( start, end );

    helperLine1 = new THREE.Line( geometry, helperLineMaterial );

    scene.add(helperLine1);
}

function drawHelperLine2(start, end)
{
    scene.remove(helperLine2);

    var geometry = new THREE.Geometry();
    geometry.vertices.push( start, end );

    helperLine2 = new THREE.Line( geometry, helperLineMaterial );

    scene.add(helperLine2);
}

function drawHelperLine3(start, end)
{
    scene.remove(helperLine3);

    var geometry = new THREE.Geometry();
    geometry.vertices.push( start, end );

    helperLine3 = new THREE.Line( geometry, helperLineMaterial );

    scene.add(helperLine3);
}

function drawHelperLine4(start, end)
{
    scene.remove(helperLine4);

    var geometry = new THREE.Geometry();
    geometry.vertices.push( start, end );

    helperLine4 = new THREE.Line( geometry, helperLineMaterial );

    scene.add(helperLine4);
}

function addHeadTarget()
{
    var bufferGeometry = new THREE.SphereGeometry( 0.02, 32, 32 );
    var texture = THREE.ImageUtils.loadTexture("qrc:/textures/resources/valter_model_json/textures/robot_eye.jpg");
    texture.minFilter = THREE.NearestFilter;
    var meshMaterial = new THREE.MeshPhongMaterial({color: "#00FF00", specular: "#555555", shininess: 25});
    meshMaterial.shading = THREE.FlatShading;

    headTarget = new THREE.Mesh( bufferGeometry, meshMaterial );

    scene.add(headTarget);

    interceptedObjects.push(headTarget);
}

function setMouseCameraRaycaster(canvas, justPressed)
{
    mouseCameraRaycaster.setFromCamera( mouse, camera );

    //object selected
    if ( selectedManipulationObject )
    {
        var intersectsPlane = mouseCameraRaycaster.intersectObject( objectManipulationPlane );
        if ( intersectsPlane.length > 0 )
        {
            var curObjectPosY = selectedManipulationObject.position.y;
            if (ctrlPressed)
            {
                selectedManipulationObject.position.copy( intersectsPlane[0].point.sub( objectManipulationOffset ) );
                selectedManipulationObject.position.y = curObjectPosY;
            }
            else
            {
                selectedManipulationObject.position.copy( intersectsPlane[0].point.sub( objectManipulationOffset ) );
            }
        }

        switch (interceptedObject)
        {
            case headTarget:
                headTrackingHeadHandler();
            break;
        }

        return;
    }

    var intersects = mouseCameraRaycaster.intersectObjects( interceptedObjects , false);

    if ( intersects.length > 0 )
    {
        interceptedObject = intersects[ 0 ].object;

        if (justPressed)
        {
            selectedManipulationObject = interceptedObject;
            var intersectsSelected = mouseCameraRaycaster.intersectObject( objectManipulationPlane , false);
            if ( intersectsSelected.length > 0 )
            {
                objectManipulationOffset.copy( intersectsSelected[0].point ).sub( objectManipulationPlane.position );
            }
            return;
        }
        else
        {
            objectManipulationPlane.position.copy( interceptedObject.position );
            objectManipulationPlane.lookAt( camera.position );

            switch (interceptedObject)
            {
                case headTarget:
                    //console.log("HIT headTarget");
                break;
            }
        }
    }
}

function headYawProjOnBodyGroupPlane()
{
    var x1 = bodyGroupHelperMeshCenterPosition.x;
    var x2 = bodyGroupHelperMeshXPosition.x;
    var x3 = bodyGroupHelperMeshCenterPosition.x;
    var x4 = bodyGroupHelperMeshZPosition.x;

    var y1 = bodyGroupHelperMeshCenterPosition.y;
    var y2 = bodyGroupHelperMeshXPosition.y;
    var y3 = bodyGroupHelperMeshCenterPosition.y;
    var y4 = bodyGroupHelperMeshZPosition.y;

    var z1 = bodyGroupHelperMeshCenterPosition.z;
    var z2 = bodyGroupHelperMeshXPosition.z;
    var z3 = bodyGroupHelperMeshCenterPosition.z;
    var z4 = bodyGroupHelperMeshZPosition.z;

    var dx1 = x2-x1;
    var dx2 = x4-x3;
    var dy1 = y2-y1;
    var dy2 = y4-y3;
    var dz1 = z2-z1;
    var dz2 = z4-z3;

    var v1 = new THREE.Vector3(dx1, dy1, dz1);
    var v2 = new THREE.Vector3(dx2, dy2, dz2);
    //bodyGroup plane normal
    var normal = v1.cross(v2).negate();

    var tVx1 = headEndEffectorPosition.x;
    var tVx2 = headTarget.position.x;

    var tVy1 = headEndEffectorPosition.y;
    var tVy2 = headTarget.position.y;

    var tVz1 = headEndEffectorPosition.z;
    var tVz2 = headTarget.position.z;

    var dtVx = tVx2 - tVx1;
    var dtVy = tVy2 - tVy1;
    var dtVz = tVz2 - tVz1;

    var proj = new THREE.Vector3(dtVx, dtVy, dtVz);
    proj.projectOnPlane(normal);
    proj.add(bodyGroupHelperMeshCenterPosition);

    drawHelperLine4(bodyGroupHelperMeshCenterPosition, proj);

    return proj;
}
