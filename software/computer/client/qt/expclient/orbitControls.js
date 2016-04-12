var target = new THREE.Vector3(0, 0, 0);

var scale = 1;

var panStart = new THREE.Vector2();
var panEnd = new THREE.Vector2();
var panDelta = new THREE.Vector2();

var panOffset = new THREE.Vector3();

// current position in spherical coordinates
var theta;
var phi;

// How far you can dolly in and out ( PerspectiveCamera only )
var minDistance = 0.5;
var maxDistance = 20;

// How far you can orbit vertically, upper and lower limits.
// Range is 0 to Math.PI radians.
var minPolarAngle = 0; // radians
var maxPolarAngle = Math.PI; // radians

// How far you can orbit horizontally, upper and lower limits.
// If set, must be a sub-interval of the interval [ - Math.PI, Math.PI ].
var minAzimuthAngle = - Infinity; // radians
var maxAzimuthAngle = Infinity; // radians

var EPS = 0.000001;

var zoomSpeed = 2.0;

var rotateStart = new THREE.Vector2();
var rotateEnd = new THREE.Vector2();
var rotateDelta = new THREE.Vector2();

var rotateSpeed = 1.0;

var phiDelta = 0;
var thetaDelta = 0;

function initOrbitControl()
{
    target.copy(valterGroup.position);
    target.y = 1.0;
}

function handleMouseDownPan( mouseX, mouseY )
{
    //console.log( 'handleMouseDownPan' );
    panStart.set( mouseX, mouseY );
}

function handleMouseMovePan( mouseX, mouseY )
{
    //console.log( 'handleMouseMovePan' );
    panEnd.set( mouseX, mouseY  );
    panDelta.subVectors( panEnd, panStart );
    pan( panDelta.x, panDelta.y );
    panStart.copy( panEnd );

    updateView();
}

function handleMouseDownRotate(  mouseX, mouseY )
{
    //console.log( 'handleMouseDownRotate' );
    rotateStart.set(  mouseX, mouseY );
}

function handleMouseMoveRotate( mouseX, mouseY)
{
    //console.log( 'handleMouseMoveRotate' );
    rotateEnd.set( mouseX, mouseY );
    rotateDelta.subVectors( rotateEnd, rotateStart );

    // rotating across whole screen goes 360 degrees around
    rotateLeft( 2 * Math.PI * rotateDelta.x / canvasWidth * rotateSpeed );

    // rotating up and down along whole screen attempts to go 360, but limited to 180
    rotateUp( 2 * Math.PI * rotateDelta.y / canvasHeight * rotateSpeed );

    rotateStart.copy( rotateEnd );
    updateView();
}

// deltaX and deltaY are in pixels; right and down are positive
var pan = function() {

    var offset = new THREE.Vector3();

    return function( deltaX, deltaY ) {

        var position = camera.position;

        if (ctrlPressed && selectedManipulationObject)
        {
            target.copy(selectedManipulationObject.position);
        }


        offset.copy( position );
        offset.sub( target );

        var targetDistance = offset.length();

        // half of the fov is center to top of screen
        targetDistance *= Math.tan( ( camera.fov / 2 ) * Math.PI / 180.0 );

        // we actually don't use [canvas.width], since perspective camera is fixed to screen height
        panLeft( 2 * deltaX * targetDistance / canvasHeight, camera.matrix );

        panUp( 2 * deltaY * targetDistance / canvasHeight, camera.matrix );
    };

}();

var panLeft = function() {

    var v = new THREE.Vector3();

    return function panLeft( distance, objectMatrix ) {

        var te = objectMatrix.elements;

        // get X column of objectMatrix
        v.set( te[ 0 ], te[ 1 ], te[ 2 ] );

        v.multiplyScalar( - distance );

        panOffset.add( v );

    };

}();

var panUp = function() {

    var v = new THREE.Vector3();

    return function panUp( distance, objectMatrix ) {

        var te = objectMatrix.elements;

        // get Y column of objectMatrix
        v.set( te[ 4 ], te[ 5 ], te[ 6 ] );

        v.multiplyScalar( distance );

        panOffset.add( v );

    };

}();

var updateView = function() {

    var offset = new THREE.Vector3();

    // so camera.up is the orbit axis
    var quat = new THREE.Quaternion().setFromUnitVectors( new THREE.Vector3( 0, 1, 0 ), new THREE.Vector3( 0, 1, 0 ) );
    var quatInverse = quat.clone().inverse();

    var lastPosition = new THREE.Vector3();
    var lastQuaternion = new THREE.Quaternion();

    return function updateView() {

        var position = camera.position;

        offset.copy( position ).sub( target );

        // rotate offset to "y-axis-is-up" space
        offset.applyQuaternion( quat );

        // angle from z-axis around y-axis
        theta = Math.atan2( offset.x, offset.z );

        // angle from y-axis
        phi = Math.atan2( Math.sqrt( offset.x * offset.x + offset.z * offset.z ), offset.y );

        theta += thetaDelta;
        phi += phiDelta;

        // restrict theta to be between desired limits
        theta = Math.max( minAzimuthAngle, Math.min( maxAzimuthAngle, theta ) );

        // restrict phi to be between desired limits
        phi = Math.max( minPolarAngle, Math.min( maxPolarAngle, phi ) );

        // restrict phi to be between EPS and PI-EPS
        phi = Math.max( EPS, Math.min( Math.PI - EPS, phi ) );

        var radius = offset.length() * scale;

        // restrict radius to be between desired limits
        radius = Math.max( minDistance, Math.min( maxDistance, radius ) );

        // move target to panned location
        target.add( panOffset );

        offset.x = radius * Math.sin( phi ) * Math.sin( theta );
        offset.y = radius * Math.cos( phi );
        offset.z = radius * Math.sin( phi ) * Math.cos( theta );

        // rotate offset back to "camera-up-vector-is-up" space
        offset.applyQuaternion( quatInverse );

        position.copy( target ).add( offset );

        camera.lookAt( target );
        helperSphere.position.copy(target);

        thetaDelta = 0;
        phiDelta = 0;
        scale = 1;
        panOffset.set( 0, 0, 0 );
    };

}();

function dollyIn()
{
    scale *= getZoomScale();
    updateView();
}


function dollyOut()
{
    scale /= getZoomScale();
    updateView();
}

function rotateLeft( angle )
{
    thetaDelta -= angle;
}

function rotateUp( angle )
{
    phiDelta -= angle;
}

function getZoomScale()
{
    return Math.pow( 0.95, zoomSpeed );
}
