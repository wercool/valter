
function loadModels()
{
    var loader = new THREE.JSONLoader();

    //leftWheelChamberMesh
    loader.load( "qrc:/json_models/resources/valter_model_json/wheel.chamber.json", function ( geometry, materials ) {
        geometry.computeVertexNormals();
        var bufferGeometry = new THREE.BufferGeometry();
        bufferGeometry.fromGeometry(geometry);
        var meshMaterial = new THREE.MeshPhongMaterial({color: "#1D1E1F", specular: "#555555", shininess: 1});
        var mesh = new THREE.Mesh( bufferGeometry, meshMaterial );
        mesh.position.z = -0.194;
        leftWheelGroup.add(mesh);
    } );

    //leftWheelMetalFrontMesh
    loader.load( "qrc:/json_models/resources/valter_model_json/wheel.metal.front.json", function ( geometry, materials ) {
        geometry.computeVertexNormals();
        var bufferGeometry = new THREE.BufferGeometry();
        bufferGeometry.fromGeometry(geometry);
        var meshMaterial = new THREE.MeshPhongMaterial({color: "#E6E9EB", specular: "#555555", shininess: 0.5});
        var mesh = new THREE.Mesh( bufferGeometry, meshMaterial );
        mesh.position.z = -0.196;
        leftWheelGroup.add(mesh);
    } );

    //leftWheelMetalInternalMesh
    loader.load( "qrc:/json_models/resources/valter_model_json/wheel.metal.internal.json", function ( geometry, materials ) {
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
    loader.load( "qrc:/json_models/resources/valter_model_json/wheel.chamber.json", function ( geometry, materials ) {
        geometry.computeVertexNormals();
        var bufferGeometry = new THREE.BufferGeometry();
        bufferGeometry.fromGeometry(geometry);
        var meshMaterial = new THREE.MeshPhongMaterial({color: "#1D1E1F", specular: "#555555", shininess: 1});
        var mesh = new THREE.Mesh( bufferGeometry, meshMaterial );
        mesh.position.z = 0.194;
        rightWheelGroup.add(mesh);
    } );

    //rightWheelMetalFrontMesh
    loader.load( "qrc:/json_models/resources/valter_model_json/wheel.metal.front.json", function ( geometry, materials ) {
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
    loader.load( "qrc:/json_models/resources/valter_model_json/wheel.metal.internal.json", function ( geometry, materials ) {
        geometry.computeVertexNormals();
        var bufferGeometry = new THREE.BufferGeometry();
        bufferGeometry.fromGeometry(geometry);
        var meshMaterial = new THREE.MeshPhongMaterial({color: "#E6E9EB", specular: "#555555", shininess: 0.5});
        var mesh = new THREE.Mesh( bufferGeometry, meshMaterial );
        mesh.position.z = 0.191;
        rightWheelGroup.add(mesh);
    } );

    //wheelAxisMesh
    loader.load( "qrc:/json_models/resources/valter_model_json/wheel.axis.json", function ( geometry, materials ) {
        geometry.computeVertexNormals();
        var bufferGeometry = new THREE.BufferGeometry();
        bufferGeometry.fromGeometry(geometry);
        var meshMaterial = new THREE.MeshPhongMaterial({color: "#9EA2A3", specular: "#555555", shininess: 0.5});
        var mesh = new THREE.Mesh( bufferGeometry, meshMaterial );
        platformGroup.add(mesh);
    } );

    //wheelAxisFramesMesh
    loader.load( "qrc:/json_models/resources/valter_model_json/wheel.axis.frames.json", function ( geometry, materials ) {
        geometry.computeVertexNormals();
        var bufferGeometry = new THREE.BufferGeometry();
        bufferGeometry.fromGeometry(geometry);
        var meshMaterial = new THREE.MeshPhongMaterial({color: "#072904", specular: "#555555", shininess: 1.5});
        var mesh = new THREE.Mesh( bufferGeometry, meshMaterial );
        platformGroup.add(mesh);
    } );

    //platformCaseMesh
    loader.load( "qrc:/json_models/resources/valter_model_json/platfrom.case.json", function ( geometry, materials ) {
        geometry.computeVertexNormals();
        var bufferGeometry = new THREE.BufferGeometry();
        bufferGeometry.fromGeometry(geometry);
        var texture = THREE.ImageUtils.loadTexture("qrc:/textures/resources/valter_model_json/textures/aluminum.jpg");
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
