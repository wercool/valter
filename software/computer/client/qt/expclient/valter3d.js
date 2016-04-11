Qt.include("qrc:/valter3d/resources/3rdparty/three.js")
Qt.include("qrc:/valter3d/utils.js")
Qt.include("qrc:/valter3d/json_models_loader.js")
Qt.include("qrc:/valter3d/orbitControls.js")


//helpers
var zeroVector = new THREE.Vector3(0, 1.0, 0);
var helperLine1 = new THREE.Line();
var helperLine2 = new THREE.Line();
var helperLine3 = new THREE.Line();
var helperLine4 = new THREE.Line();


var helperSphere = new THREE.SphereGeometry( 0.01, 32, 32 );

var mouse = new THREE.Vector2(0,0);
var mouseCameraRaycaster = new THREE.Raycaster();

var interceptedObjects = [];
var interceptedObject;
var objectManipulationPlane;
var objectManipulationOffset = new THREE.Vector3();
var selectedManipulationObject;

var camera, scene, renderer;
var light1, light2, light3;

var sceneInit = false;
var sceneAfterInit = false;
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
var manEndEffectorMeshPosition = new THREE.Vector3();

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
var headGroup = new THREE.Object3D();
var headYawGroupHelperMesh = new THREE.Mesh();
var headYawGroupNormalHelperMesh = new THREE.Mesh();
var headYawGroupHelperMeshPosition = new THREE.Vector3();
var headYawGroupNormalHelperMeshPosition = new THREE.Vector3();
var headYawToHeadTargetVectorXZProjected = new THREE.Vector3();

var headEndEffectorGroup = new THREE.Object3D();
var headGroupHeadTargetLine = new THREE.Line();
var headGroupHeadTargetLine3 = new THREE.Line3();
var headEndEffectorGroupVectorMesh, headEndEffectorGroupVectorHelperMesh;
var headEndEffectorPosition, headEndEffectorHelperPosition;
var headTarget = new THREE.Mesh();
var resetHeadTilt = false;
var resetHeadYaw = false;

var valterGroup = new THREE.Object3D();
var valterGroupXLineHelperMesh = new THREE.Mesh();
var valterGroupZLineHelperMesh = new THREE.Mesh();
var valterGroupXLine = new THREE.Line();
var valterGroupXLineHelperMeshPosition = new THREE.Vector3();
var valterGroupZLine = new THREE.Line();

//bodyGroup Helpers
var bodyGroupHelperMeshCenter = new THREE.Mesh();
var bodyGroupHelperMeshX = new THREE.Mesh();
var bodyGroupHelperMeshZ = new THREE.Mesh();

var bodyGroupHelperMeshCenterPosition = new THREE.Vector3();
var bodyGroupHelperMeshXPosition = new THREE.Vector3();
var bodyGroupHelperMeshZPosition = new THREE.Vector3();

/*
  various current values
*/
var valterGroupRotation = 0.0;

function paintGL(canvas)
{
    switch (canvas.mouseButtonPressed)
    {
        case 1:
            if ( !selectedManipulationObject )
            {
                var pos = mousePointerXYToSceneXZ(canvas);
                drawPixel(pos);
            }
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
            bodyGroup.rotation.z = -0.5;
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
//            headYawGroup.rotation.y = -0.5;
//            headGroup.rotation.z = -0.5;

            sceneInit = true;
        }
        else
        {
            if (!sceneAfterInit)
            {
                addValterGroupHelpers();
                addHeadTarget();

                headTarget.position.copy(headEndEffectorGroup.localToWorld(headEndEffectorGroupVectorHelperMesh.position.clone()));

                sceneAfterInit = true;
            }
            else
            {
                if (!manEndEffectorMeshPosition.equals ( manEndEffectorGroup.localToWorld(manEndEffectorMesh.position.clone()) ))
                {
                    drawPixel(manEndEffectorMeshPosition);
                }

                manEndEffectorMeshPosition = manEndEffectorGroup.localToWorld(manEndEffectorMesh.position.clone());

                headEndEffectorPosition = headEndEffectorGroup.localToWorld(headEndEffectorGroupVectorMesh.position.clone());
                headEndEffectorHelperPosition = headEndEffectorGroup.localToWorld(headEndEffectorGroupVectorHelperMesh.position.clone());

                headYawGroupHelperMeshPosition = headYawGroup.localToWorld(headYawGroupHelperMesh.position.clone());
                headYawGroupNormalHelperMeshPosition = headYawGroup.localToWorld(headYawGroupNormalHelperMesh.position.clone());

                valterGroupXLineHelperMeshPosition = valterGroup.localToWorld(valterGroupXLineHelperMesh.position.clone());

                bodyGroupHelperMeshCenterPosition = bodyGroup.localToWorld(bodyGroupHelperMeshCenter.position.clone());
                bodyGroupHelperMeshXPosition = bodyGroup.localToWorld(bodyGroupHelperMeshX.position.clone());
                bodyGroupHelperMeshZPosition = bodyGroup.localToWorld(bodyGroupHelperMeshZ.position.clone());

                drawHeadGroupHeadTargetLine();
                renderValterGroupHelpers();
                headYawProjOnBodyGroupPlane();

                /* HELPERS */
                drawHelperLine1(headYawGroupHelperMeshPosition, headYawGroupNormalHelperMeshPosition);
                drawHelperLine2(bodyGroupHelperMeshCenterPosition, bodyGroupHelperMeshZPosition);
                drawHelperLine3(bodyGroupHelperMeshCenterPosition, bodyGroupHelperMeshXPosition);
            }
        }
    }

    canvas.updateLabels();

    renderer.render( scene, camera );
}

function setValterGroupPositionDxDz(dx, dz)
{
    valterGroup.position.x += dx * Math.cos(valterGroup.rotation.y);
    valterGroup.position.z -= dz * Math.sin(valterGroup.rotation.y);
    headTarget.position.copy(headEndEffectorGroup.localToWorld(headEndEffectorGroupVectorHelperMesh.position.clone()));

    var supportWheelsRotDy = 0;
    if (frontLeftSupportWheelGroup.rotation.y > 0)
    {
        supportWheelsRotDy = -0.07;
    }
    else
    {
        supportWheelsRotDy = 0.07;
    }

    if (Math.abs(frontLeftSupportWheelGroup.rotation.y) > 0.07)
    {
        frontLeftSupportWheelGroup.rotation.y   += supportWheelsRotDy;
        frontRightSupportWheelGroup.rotation.y  += supportWheelsRotDy;
        rearLeftSupportWheelGroup.rotation.y    += -supportWheelsRotDy;
        rearRightSupportWheelGroup.rotation.y   += -supportWheelsRotDy;
    }
    else
    {
        frontLeftSupportWheelGroup.rotation.y   = 0;
        frontRightSupportWheelGroup.rotation.y  = 0;
        rearLeftSupportWheelGroup.rotation.y    = 0;
        rearRightSupportWheelGroup.rotation.y   = 0;
    }
}

function setValterGroupRotationDy(da)
{
    valterGroup.rotation.y += da;
    headTarget.position.copy(headEndEffectorGroup.localToWorld(headEndEffectorGroupVectorHelperMesh.position.clone()));

    valterGroupRotation = valterGroup.rotation.y;

    var supportWheelsRotY = frontLeftSupportWheelGroup.rotation.y + da * 4;

    if ( (supportWheelsRotY < Math.PI / 2 && da > 0) || (supportWheelsRotY > -Math.PI / 2 && da < 0))
    {
        frontLeftSupportWheelGroup.rotation.y = supportWheelsRotY;
        frontRightSupportWheelGroup.rotation.y = supportWheelsRotY;
        rearLeftSupportWheelGroup.rotation.y = -supportWheelsRotY;
        rearRightSupportWheelGroup.rotation.y = -supportWheelsRotY;
    }
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

function tilitHead()
{
//    var dy = headEndEffectorPosition.y - headTarget.position.y;
//    var l = headGroupHeadTargetLine3.distance();
//    var alpha = -Math.asin(dy / l) - bodyGroup.rotation.z;

    var x1 = headYawGroupHelperMeshPosition.x;
    var x2 = headYawGroupNormalHelperMeshPosition.x;
    var x3 = headEndEffectorPosition.x;
    var x4 = headTarget.position.x;

    var y1 = headYawGroupHelperMeshPosition.y;
    var y2 = headYawGroupNormalHelperMeshPosition.y;
    var y3 = headEndEffectorPosition.y;
    var y4 = headTarget.position.y;

    var z1 = headYawGroupHelperMeshPosition.z;
    var z2 = headYawGroupNormalHelperMeshPosition.z;
    var z3 = headEndEffectorPosition.z;
    var z4 = headTarget.position.z;

    var dx1 = x2-x1;
    var dx2 = x4-x3;
    var dy1 = y2-y1;
    var dy2 = y4-y3;
    var dz1 = z2-z1;
    var dz2 = z4-z3;

    var v1 = new THREE.Vector3(dx1, dy1, dz1);
    var v2 = new THREE.Vector3(dx2, dy2, dz2);
    var alpha = v1.angleTo(v2);
    var sign = v1.cross(v2).z;
    alpha *= ((sign > 0) ? 1 : -1);

    //console.log(alpha);

    if (alpha > 0 || alpha < -degToRad(60))
    {
        resetHeadTilt = true;
        if (alpha > 0)
        {
            headGroup.rotation.z = 0;
        }
        else if (alpha < -degToRad(60))
        {
            headGroup.rotation.z = -degToRad(60);
        }
        console.log("resetHeadTilt");
    }
    else
    {
        headGroup.rotation.z = alpha;
        resetHeadTilt = false;
    }
    headEndEffectorGroupVectorHelperMesh.position.x = headGroupHeadTargetLine3.distance();
}

function yawHead()
{


    //console.log(alpha);


//    var x1 = valterGroup.position.x;
//    var x2 = valterGroupXLineHelperMeshPosition.x;
//    var x3 = valterGroup.position.x;
//    var x4 = headYawToHeadTargetVectorXZProjected.x;

//    var z1 = valterGroup.position.z;
//    var z2 = valterGroupXLineHelperMeshPosition.z;
//    var z3 = valterGroup.position.z;
//    var z4 = headYawToHeadTargetVectorXZProjected.z;

//    var dx1 = x2-x1;
//    var dz1 = z2-z1;
//    var dx2 = x4-x3;
//    var dz2 = z4-z3;

//    var alpha = Math.atan2(dz2, dx2) - Math.atan2(dz1, dx1);

//    var dir = new THREE.Vector3(dx1, 0, dz1);
//    var prj = new THREE.Vector3(dx2, 0, dz2);
//    var c = new THREE.Vector3();
//    var sign = (c.crossVectors( dir, prj ).y > 0 ? 1 : -1);
//    var alpha = sign * dir.angleTo(prj);

    //console.log(alpha);

//    if (alpha < -1.2 + trunkGroup.rotation.y || alpha > 1.2 + trunkGroup.rotation.y)
//    {
//        resetHeadYaw = true;
//        console.log("resetHeadYaw");
//    }
//    else
//    {
//        headYawGroup.rotation.y = -alpha - trunkGroup.rotation.y;
//        resetHeadYaw = false;
//        headEndEffectorGroupVectorHelperMesh.position.x = headGroupHeadTargetLine3.distance();
//    }
}
