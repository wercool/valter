
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
        var meshMaterial = new THREE.MeshPhongMaterial({color: "#E6E9EB", specular: "#555555", shininess: 30});
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
        var meshMaterial = new THREE.MeshPhongMaterial({color: "#E6E9EB", specular: "#555555", shininess: 30});
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
        var meshMaterial = new THREE.MeshPhongMaterial({color: "#E6E9EB", specular: "#555555", shininess: 30});
        var mesh = new THREE.Mesh( bufferGeometry, meshMaterial );
        mesh.position.z = 0.191;
        rightWheelGroup.add(mesh);
    } );

    //wheelAxisMesh
    loader.load( "qrc:/json_models/resources/valter_model_json/wheel.axis.json", function ( geometry, materials ) {
        geometry.computeVertexNormals();
        var bufferGeometry = new THREE.BufferGeometry();
        bufferGeometry.fromGeometry(geometry);
        var meshMaterial = new THREE.MeshPhongMaterial({color: "#9EA2A3", specular: "#555555", shininess: 30});
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
                                                         bumpScale: 0.005,
                                                         shininess: 5
                                                     });
        meshMaterial.shading = THREE.FlatShading;
        var mesh = new THREE.Mesh( bufferGeometry, meshMaterial );
        mesh.position.y = 0.0345;
        platformGroup.add(mesh);
    } );

    //computer.case.json
    loader.load( "qrc:/json_models/resources/valter_model_json/computer.case.json", function ( geometry, materials ) {
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
        mesh.position.x = -0.27;
        mesh.position.y = 0.148;
        mesh.rotation.y = degToRad(180);
        mesh.rotation.x = degToRad(-90);

        platformGroup.add(mesh);
    } );

    //support-wheel-p2.json ----------- front left
    loader.load( "qrc:/json_models/resources/valter_model_json/support-wheel-p2.json", function ( geometry, materials ) {
        geometry.computeVertexNormals();
        var bufferGeometry = new THREE.BufferGeometry();
        bufferGeometry.fromGeometry(geometry);
        var meshMaterial = new THREE.MeshPhongMaterial({color: "#E6E9EB", specular: "#555555", shininess: 0.5});
        meshMaterial.shading = THREE.FlatShading;
        var mesh = new THREE.Mesh( bufferGeometry, meshMaterial );
        mesh.position.y = 0.16 - baseShiftY;
        mesh.position.x = 0.22;
        mesh.position.z = -0.1;
        platformGroup.add(mesh);
    } );

    //support-wheel-p2.json ----------- front right
    loader.load( "qrc:/json_models/resources/valter_model_json/support-wheel-p2.json", function ( geometry, materials ) {
        geometry.computeVertexNormals();
        var bufferGeometry = new THREE.BufferGeometry();
        bufferGeometry.fromGeometry(geometry);
        var meshMaterial = new THREE.MeshPhongMaterial({color: "#E6E9EB", specular: "#555555", shininess: 0.5});
        meshMaterial.shading = THREE.FlatShading;
        var mesh = new THREE.Mesh( bufferGeometry, meshMaterial );
        mesh.position.y = 0.16 - baseShiftY;
        mesh.position.x = 0.22;
        mesh.position.z = 0.1;
        platformGroup.add(mesh);
    } );

    //support-wheel-p2.json ----------- rear left
    loader.load( "qrc:/json_models/resources/valter_model_json/support-wheel-p2.json", function ( geometry, materials ) {
        geometry.computeVertexNormals();
        var bufferGeometry = new THREE.BufferGeometry();
        bufferGeometry.fromGeometry(geometry);
        var meshMaterial = new THREE.MeshPhongMaterial({color: "#E6E9EB", specular: "#555555", shininess: 0.5});
        meshMaterial.shading = THREE.FlatShading;
        var mesh = new THREE.Mesh( bufferGeometry, meshMaterial );
        mesh.position.y = 0.16 - baseShiftY;
        mesh.position.x = -0.22;
        mesh.position.z = -0.1;
        platformGroup.add(mesh);
    } );

    //support-wheel-p2.json ----------- rear right
    loader.load( "qrc:/json_models/resources/valter_model_json/support-wheel-p2.json", function ( geometry, materials ) {
        geometry.computeVertexNormals();
        var bufferGeometry = new THREE.BufferGeometry();
        bufferGeometry.fromGeometry(geometry);
        var meshMaterial = new THREE.MeshPhongMaterial({color: "#E6E9EB", specular: "#555555", shininess: 0.5});
        meshMaterial.shading = THREE.FlatShading;
        var mesh = new THREE.Mesh( bufferGeometry, meshMaterial );
        mesh.position.y = 0.16 - baseShiftY;
        mesh.position.x = -0.22;
        mesh.position.z = 0.1;
        platformGroup.add(mesh);
    } );

    //----------------------------------------------------frontLeftSupportWheelGroup
    //support-wheel-p1.json
    loader.load( "qrc:/json_models/resources/valter_model_json/support-wheel-p1.json", function ( geometry, materials ) {
        geometry.computeVertexNormals();
        var bufferGeometry = new THREE.BufferGeometry();
        bufferGeometry.fromGeometry(geometry);
        var meshMaterial = new THREE.MeshPhongMaterial({color: "#E6E9EB", specular: "#555555", shininess: 0.5});
        meshMaterial.shading = THREE.FlatShading;
        var mesh = new THREE.Mesh( bufferGeometry, meshMaterial );

        frontLeftSupportWheelGroup.position.x = 0.22;
        frontLeftSupportWheelGroup.position.y = mesh.position.y = 0.065 - baseShiftY;
        frontLeftSupportWheelGroup.position.z = -0.1;

        mesh.position.y = 0;
        mesh.position.x = -0.041;
        mesh.position.z = 0;

        frontLeftSupportWheelGroup.add(mesh);
    } );

    //wheel1.p1.json
    loader.load( "qrc:/json_models/resources/valter_model_json/wheel1.p1.json", function ( geometry, materials ) {
        geometry.computeVertexNormals();
        var bufferGeometry = new THREE.BufferGeometry();
        bufferGeometry.fromGeometry(geometry);
        var meshMaterial = new THREE.MeshPhongMaterial({color: "#1D1E1F", specular: "#555555", shininess: 0.5});
        meshMaterial.shading = THREE.FlatShading;
        var mesh = new THREE.Mesh( bufferGeometry, meshMaterial );

        mesh.position.y = 0;
        mesh.position.x = -0.041;
        mesh.position.z = 0;

        frontLeftSupportWheelGroup.add(mesh);
    } );

    //wheel1.p2.json
    loader.load( "qrc:/json_models/resources/valter_model_json/wheel1.p2.json", function ( geometry, materials ) {
        geometry.computeVertexNormals();
        var bufferGeometry = new THREE.BufferGeometry();
        bufferGeometry.fromGeometry(geometry);
        var meshMaterial = new THREE.MeshPhongMaterial({color: "#E6E9EB", specular: "#555555", shininess: 0.5});
        var mesh = new THREE.Mesh( bufferGeometry, meshMaterial );

        mesh.position.y = 0;
        mesh.position.x = -0.041;
        mesh.position.z = 0.008;

        frontLeftSupportWheelGroup.add(mesh);
    } );
    //wheel1.p2.json
    loader.load( "qrc:/json_models/resources/valter_model_json/wheel1.p2.json", function ( geometry, materials ) {
        geometry.computeVertexNormals();
        var bufferGeometry = new THREE.BufferGeometry();
        bufferGeometry.fromGeometry(geometry);
        var meshMaterial = new THREE.MeshPhongMaterial({color: "#E6E9EB", specular: "#555555", shininess: 0.5});
        meshMaterial.shading = THREE.FlatShading;
        var mesh = new THREE.Mesh( bufferGeometry, meshMaterial );

        mesh.rotation.y = degToRad(180);

        mesh.position.y = 0;
        mesh.position.x = -0.041;
        mesh.position.z = -0.008;

        frontLeftSupportWheelGroup.add(mesh);
    } );

    //----------------------------------------------------frontRightSupportWheelGroup
    //support-wheel-p1.json
    loader.load( "qrc:/json_models/resources/valter_model_json/support-wheel-p1.json", function ( geometry, materials ) {
        geometry.computeVertexNormals();
        var bufferGeometry = new THREE.BufferGeometry();
        bufferGeometry.fromGeometry(geometry);
        var meshMaterial = new THREE.MeshPhongMaterial({color: "#E6E9EB", specular: "#555555", shininess: 0.5});
        meshMaterial.shading = THREE.FlatShading;
        var mesh = new THREE.Mesh( bufferGeometry, meshMaterial );

        frontRightSupportWheelGroup.position.x = 0.22;
        frontRightSupportWheelGroup.position.y = mesh.position.y = 0.065 - baseShiftY;
        frontRightSupportWheelGroup.position.z = 0.1;

        mesh.position.y = 0;
        mesh.position.x = -0.041;
        mesh.position.z = 0;

        frontRightSupportWheelGroup.add(mesh);
    } );

    //wheel1.p1.json
    loader.load( "qrc:/json_models/resources/valter_model_json/wheel1.p1.json", function ( geometry, materials ) {
        geometry.computeVertexNormals();
        var bufferGeometry = new THREE.BufferGeometry();
        bufferGeometry.fromGeometry(geometry);
        var meshMaterial = new THREE.MeshPhongMaterial({color: "#1D1E1F", specular: "#555555", shininess: 0.5});
        meshMaterial.shading = THREE.FlatShading;
        var mesh = new THREE.Mesh( bufferGeometry, meshMaterial );

        mesh.position.y = 0;
        mesh.position.x = -0.041;
        mesh.position.z = 0;

        frontRightSupportWheelGroup.add(mesh);
    } );

    //wheel1.p2.json
    loader.load( "qrc:/json_models/resources/valter_model_json/wheel1.p2.json", function ( geometry, materials ) {
        geometry.computeVertexNormals();
        var bufferGeometry = new THREE.BufferGeometry();
        bufferGeometry.fromGeometry(geometry);
        var meshMaterial = new THREE.MeshPhongMaterial({color: "#E6E9EB", specular: "#555555", shininess: 0.5});
        var mesh = new THREE.Mesh( bufferGeometry, meshMaterial );

        mesh.position.y = 0;
        mesh.position.x = -0.041;
        mesh.position.z = 0.008;

        frontRightSupportWheelGroup.add(mesh);
    } );
    //wheel1.p2.json
    loader.load( "qrc:/json_models/resources/valter_model_json/wheel1.p2.json", function ( geometry, materials ) {
        geometry.computeVertexNormals();
        var bufferGeometry = new THREE.BufferGeometry();
        bufferGeometry.fromGeometry(geometry);
        var meshMaterial = new THREE.MeshPhongMaterial({color: "#E6E9EB", specular: "#555555", shininess: 0.5});
        meshMaterial.shading = THREE.FlatShading;
        var mesh = new THREE.Mesh( bufferGeometry, meshMaterial );

        mesh.rotation.y = degToRad(180);

        mesh.position.y = 0;
        mesh.position.x = -0.041;
        mesh.position.z = -0.008;

        frontRightSupportWheelGroup.add(mesh);
    } );


    //----------------------------------------------------rearLeftSupportWheelGroup
    //support-wheel-p1.json
    loader.load( "qrc:/json_models/resources/valter_model_json/support-wheel-p1.json", function ( geometry, materials ) {
        geometry.computeVertexNormals();
        var bufferGeometry = new THREE.BufferGeometry();
        bufferGeometry.fromGeometry(geometry);
        var meshMaterial = new THREE.MeshPhongMaterial({color: "#E6E9EB", specular: "#555555", shininess: 0.5});
        meshMaterial.shading = THREE.FlatShading;
        var mesh = new THREE.Mesh( bufferGeometry, meshMaterial );

        rearLeftSupportWheelGroup.position.x = -0.22;
        rearLeftSupportWheelGroup.position.y = mesh.position.y = 0.065 - baseShiftY;
        rearLeftSupportWheelGroup.position.z = 0.1;

        mesh.position.y = 0;
        mesh.position.x = -0.041;
        mesh.position.z = 0;

        rearLeftSupportWheelGroup.add(mesh);
    } );

    //wheel1.p1.json
    loader.load( "qrc:/json_models/resources/valter_model_json/wheel1.p1.json", function ( geometry, materials ) {
        geometry.computeVertexNormals();
        var bufferGeometry = new THREE.BufferGeometry();
        bufferGeometry.fromGeometry(geometry);
        var meshMaterial = new THREE.MeshPhongMaterial({color: "#1D1E1F", specular: "#555555", shininess: 0.5});
        meshMaterial.shading = THREE.FlatShading;
        var mesh = new THREE.Mesh( bufferGeometry, meshMaterial );

        mesh.position.y = 0;
        mesh.position.x = -0.041;
        mesh.position.z = 0;

        rearLeftSupportWheelGroup.add(mesh);
    } );

    //wheel1.p2.json
    loader.load( "qrc:/json_models/resources/valter_model_json/wheel1.p2.json", function ( geometry, materials ) {
        geometry.computeVertexNormals();
        var bufferGeometry = new THREE.BufferGeometry();
        bufferGeometry.fromGeometry(geometry);
        var meshMaterial = new THREE.MeshPhongMaterial({color: "#E6E9EB", specular: "#555555", shininess: 0.5});
        var mesh = new THREE.Mesh( bufferGeometry, meshMaterial );

        mesh.position.y = 0;
        mesh.position.x = -0.041;
        mesh.position.z = 0.008;

        rearLeftSupportWheelGroup.add(mesh);
    } );
    //wheel1.p2.json
    loader.load( "qrc:/json_models/resources/valter_model_json/wheel1.p2.json", function ( geometry, materials ) {
        geometry.computeVertexNormals();
        var bufferGeometry = new THREE.BufferGeometry();
        bufferGeometry.fromGeometry(geometry);
        var meshMaterial = new THREE.MeshPhongMaterial({color: "#E6E9EB", specular: "#555555", shininess: 0.5});
        meshMaterial.shading = THREE.FlatShading;
        var mesh = new THREE.Mesh( bufferGeometry, meshMaterial );

        mesh.rotation.y = degToRad(180);

        mesh.position.y = 0;
        mesh.position.x = -0.041;
        mesh.position.z = -0.008;

        rearLeftSupportWheelGroup.add(mesh);
    } );


    //----------------------------------------------------rearRightSupportWheelGroup
    //support-wheel-p1.json
    loader.load( "qrc:/json_models/resources/valter_model_json/support-wheel-p1.json", function ( geometry, materials ) {
        geometry.computeVertexNormals();
        var bufferGeometry = new THREE.BufferGeometry();
        bufferGeometry.fromGeometry(geometry);
        var meshMaterial = new THREE.MeshPhongMaterial({color: "#E6E9EB", specular: "#555555", shininess: 0.5});
        meshMaterial.shading = THREE.FlatShading;
        var mesh = new THREE.Mesh( bufferGeometry, meshMaterial );

        rearRightSupportWheelGroup.position.x = -0.22;
        rearRightSupportWheelGroup.position.y = mesh.position.y = 0.065 - baseShiftY;
        rearRightSupportWheelGroup.position.z = -0.1;

        mesh.position.y = 0;
        mesh.position.x = -0.041;
        mesh.position.z = 0;

        rearRightSupportWheelGroup.add(mesh);
    } );

    //wheel1.p1.json
    loader.load( "qrc:/json_models/resources/valter_model_json/wheel1.p1.json", function ( geometry, materials ) {
        geometry.computeVertexNormals();
        var bufferGeometry = new THREE.BufferGeometry();
        bufferGeometry.fromGeometry(geometry);
        var meshMaterial = new THREE.MeshPhongMaterial({color: "#1D1E1F", specular: "#555555", shininess: 0.5});
        meshMaterial.shading = THREE.FlatShading;
        var mesh = new THREE.Mesh( bufferGeometry, meshMaterial );

        mesh.position.y = 0;
        mesh.position.x = -0.041;
        mesh.position.z = 0;

        rearRightSupportWheelGroup.add(mesh);
    } );

    //wheel1.p2.json
    loader.load( "qrc:/json_models/resources/valter_model_json/wheel1.p2.json", function ( geometry, materials ) {
        geometry.computeVertexNormals();
        var bufferGeometry = new THREE.BufferGeometry();
        bufferGeometry.fromGeometry(geometry);
        var meshMaterial = new THREE.MeshPhongMaterial({color: "#E6E9EB", specular: "#555555", shininess: 0.5});
        var mesh = new THREE.Mesh( bufferGeometry, meshMaterial );

        mesh.position.y = 0;
        mesh.position.x = -0.041;
        mesh.position.z = 0.008;

        rearRightSupportWheelGroup.add(mesh);
    } );
    //wheel1.p2.json
    loader.load( "qrc:/json_models/resources/valter_model_json/wheel1.p2.json", function ( geometry, materials ) {
        geometry.computeVertexNormals();
        var bufferGeometry = new THREE.BufferGeometry();
        bufferGeometry.fromGeometry(geometry);
        var meshMaterial = new THREE.MeshPhongMaterial({color: "#E6E9EB", specular: "#555555", shininess: 0.5});
        meshMaterial.shading = THREE.FlatShading;
        var mesh = new THREE.Mesh( bufferGeometry, meshMaterial );

        mesh.rotation.y = degToRad(180);

        mesh.position.y = 0;
        mesh.position.x = -0.041;
        mesh.position.z = -0.008;

        rearRightSupportWheelGroup.add(mesh);
    } );

    //man.p3.json
    loader.load( "qrc:/json_models/resources/valter_model_json/man.p3.json", function ( geometry, materials ) {
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
        mesh.rotation.y = degToRad(180);
        mesh.position.y = 0.546;
        mesh.position.x = 0.134;

        manLink1Group.position.y = 0.546;
        manLink1Group.position.x = 0.134;

        platformGroup.add(mesh);
    } );

    //---------------------------------------------------manLink1Group
    //man.p2.json
    loader.load( "qrc:/json_models/resources/valter_model_json/man.p2.json", function ( geometry, materials ) {
        geometry.computeVertexNormals();
        var bufferGeometry = new THREE.BufferGeometry();
        bufferGeometry.fromGeometry(geometry);
        var texture = THREE.ImageUtils.loadTexture("qrc:/textures/resources/valter_model_json/textures/aluminum.jpg");
        texture.minFilter = THREE.NearestFilter;
        var meshMaterial = new THREE.MeshPhongMaterial({
                                                         map: texture,
                                                         bumpMap: texture,
                                                         bumpScale: 0.002,
                                                         shininess: 1.5
                                                     });
        meshMaterial.shading = THREE.FlatShading;
        var mesh = new THREE.Mesh( bufferGeometry, meshMaterial );

        manLink1Group.add(mesh);
    } );

    //man.p1.json
    loader.load( "qrc:/json_models/resources/valter_model_json/man.p1.json", function ( geometry, materials ) {
        geometry.computeVertexNormals();
        var bufferGeometry = new THREE.BufferGeometry();
        bufferGeometry.fromGeometry(geometry);
        var texture = THREE.ImageUtils.loadTexture("qrc:/textures/resources/valter_model_json/textures/aluminum.jpg");
        texture.minFilter = THREE.NearestFilter;
        var meshMaterial = new THREE.MeshPhongMaterial({
                                                         map: texture,
                                                         bumpMap: texture,
                                                         bumpScale: 0.002,
                                                         shininess: 1.5
                                                     });
        meshMaterial.shading = THREE.FlatShading;
        var mesh = new THREE.Mesh( bufferGeometry, meshMaterial );

        mesh.position.x = 0.107;
        mesh.position.y = -0.49795 + 0.006;

        manLink2Group.position.x = mesh.position.x;
        manLink2Group.position.y = mesh.position.y;
        manLink2Group.rotation.y = degToRad(180);

        manLink1Group.add(manLink2Group);



        manLink1Group.add(mesh);
    } );

    //-----------------------------------------------manLink2Group
    //man.p5.json
    loader.load( "qrc:/json_models/resources/valter_model_json/man.p5.json", function ( geometry, materials ) {
        geometry.computeVertexNormals();
        var bufferGeometry = new THREE.BufferGeometry();
        bufferGeometry.fromGeometry(geometry);
        var texture = THREE.ImageUtils.loadTexture("qrc:/textures/resources/valter_model_json/textures/aluminum.jpg");
        texture.minFilter = THREE.NearestFilter;
        var meshMaterial = new THREE.MeshPhongMaterial({
                                                         map: texture,
                                                         bumpMap: texture,
                                                         bumpScale: 0.002,
                                                         shininess: 1.5
                                                     });
        meshMaterial.shading = THREE.FlatShading;
        var mesh = new THREE.Mesh( bufferGeometry, meshMaterial );

        manLink2Group.add(mesh);
    } );

    //-----------------------------------------------manLink2Group
    //man.p7_man.p8.json
    loader.load( "qrc:/json_models/resources/valter_model_json/man.p7_man.p8.json", function ( geometry, materials ) {
        geometry.computeVertexNormals();
        var bufferGeometry = new THREE.BufferGeometry();
        bufferGeometry.fromGeometry(geometry);
        var texture = THREE.ImageUtils.loadTexture("qrc:/textures/resources/valter_model_json/textures/aluminum.jpg");
        texture.minFilter = THREE.NearestFilter;
        var meshMaterial = new THREE.MeshPhongMaterial({
                                                         map: texture,
                                                         bumpMap: texture,
                                                         bumpScale: 0.002,
                                                         shininess: 1.5
                                                     });
        meshMaterial.shading = THREE.FlatShading;
        var mesh = new THREE.Mesh( bufferGeometry, meshMaterial );

        mesh.rotation.y = degToRad(-90);
        mesh.position.y = 0.412;

        manLink2Group.add(mesh);

        manLink2Group.add(manGripperTiltGroup);

        manGripperTiltGroup.position.y = mesh.position.y + 0.142;
        manGripperTiltGroup.position.x = mesh.position.x + 0.0128;
    } );

    //-----------------------------------------------manGripperTiltGroup
    //man.p9.json
    loader.load( "qrc:/json_models/resources/valter_model_json/man.p9.json", function ( geometry, materials ) {
        geometry.computeVertexNormals();
        var bufferGeometry = new THREE.BufferGeometry();
        bufferGeometry.fromGeometry(geometry);
        var texture = THREE.ImageUtils.loadTexture("qrc:/textures/resources/valter_model_json/textures/mangripperrotator.jpg");
        texture.minFilter = THREE.NearestFilter;
        var meshMaterial = new THREE.MeshPhongMaterial({
                                                         map: texture,
                                                         bumpMap: texture,
                                                         bumpScale: 0.002,
                                                         shininess: 1.5
                                                     });
        meshMaterial.shading = THREE.FlatShading;
        var mesh = new THREE.Mesh( bufferGeometry, meshMaterial );

        mesh.rotation.x = degToRad(-90);
        mesh.rotation.y = degToRad(180);

        manGripperTiltGroup.add(mesh);

        manGripperTiltGroup.add(manGripperRotateGroup);

        manGripperRotateGroup.position.y = mesh.position.y + 0.005;
        manEndEffectorGroup.position.y = mesh.position.y + 0.24;
    } );

    //-----------------------------------------------manGripperRotateGroup
    //man.p11.json
    loader.load( "qrc:/json_models/resources/valter_model_json/man.p11.json", function ( geometry, materials ) {
        geometry.computeVertexNormals();
        var bufferGeometry = new THREE.BufferGeometry();
        bufferGeometry.fromGeometry(geometry);
        var texture = THREE.ImageUtils.loadTexture("qrc:/textures/resources/valter_model_json/textures/aluminum.jpg");
        texture.minFilter = THREE.NearestFilter;
        var meshMaterial = new THREE.MeshPhongMaterial({
                                                         map: texture,
                                                         bumpMap: texture,
                                                         bumpScale: 0.002,
                                                         shininess: 1.5
                                                     });
        meshMaterial.shading = THREE.FlatShading;
        var mesh = new THREE.Mesh( bufferGeometry, meshMaterial );

        manGripperRotateGroup.add(mesh);

        manGripperRotateGroup.add(manEndEffectorGroup);
    } );

    //man.p12.json
    loader.load( "qrc:/json_models/resources/valter_model_json/man.p12.json", function ( geometry, materials ) {
        geometry.computeVertexNormals();
        var bufferGeometry = new THREE.BufferGeometry();
        bufferGeometry.fromGeometry(geometry);
        var texture = THREE.ImageUtils.loadTexture("qrc:/textures/resources/valter_model_json/textures/aluminum.jpg");
        texture.minFilter = THREE.NearestFilter;
        var meshMaterial = new THREE.MeshPhongMaterial({
                                                         map: texture,
                                                         bumpMap: texture,
                                                         bumpScale: 0.002,
                                                         shininess: 1.5
                                                     });
        meshMaterial.shading = THREE.FlatShading;
        var mesh = new THREE.Mesh( bufferGeometry, meshMaterial );

        mesh.position.y = 0.091;
        mesh.position.z = 0.05;

        manGripperRotateGroup.add(mesh);
    } );

    //man.p12.json
    loader.load( "qrc:/json_models/resources/valter_model_json/man.p12.json", function ( geometry, materials ) {
        geometry.computeVertexNormals();
        var bufferGeometry = new THREE.BufferGeometry();
        bufferGeometry.fromGeometry(geometry);
        var texture = THREE.ImageUtils.loadTexture("qrc:/textures/resources/valter_model_json/textures/aluminum.jpg");
        texture.minFilter = THREE.NearestFilter;
        var meshMaterial = new THREE.MeshPhongMaterial({
                                                         map: texture,
                                                         bumpMap: texture,
                                                         bumpScale: 0.002,
                                                         shininess: 1.5
                                                     });
        meshMaterial.shading = THREE.FlatShading;
        var mesh = new THREE.Mesh( bufferGeometry, meshMaterial );

        mesh.position.y = 0.091;
        mesh.position.z = -0.05;

        mesh.rotation.y = degToRad(180);

        manGripperRotateGroup.add(mesh);
    } );

    //-----------------------------------------------manEndEffectorGroup
    //man.end.effector.json
    loader.load( "qrc:/json_models/resources/valter_model_json/man.end.effector.json", function ( geometry, materials ) {
        geometry.computeVertexNormals();
        var bufferGeometry = new THREE.BufferGeometry();
        bufferGeometry.fromGeometry(geometry);
        var meshMaterial = new THREE.MeshPhongMaterial({color: "#FF0000", specular: "#555555", shininess: 1.0});
        manEndEffectorMesh = new THREE.Mesh( bufferGeometry, meshMaterial );

        manEndEffectorGroup.add(manEndEffectorMesh);
    } );

    //-------------------------------------------------trunkGroup
    //trunk.json
    loader.load( "qrc:/json_models/resources/valter_model_json/trunk.json", function ( geometry, materials ) {
        geometry.computeVertexNormals();
        var bufferGeometry = new THREE.BufferGeometry();
        bufferGeometry.fromGeometry(geometry);
        var texture = THREE.ImageUtils.loadTexture("qrc:/textures/resources/valter_model_json/textures/aluminum_light.jpg");
        texture.minFilter = THREE.NearestFilter;
        var meshMaterial = new THREE.MeshPhongMaterial({
                                                         map: texture,
                                                         bumpMap: texture,
                                                         bumpScale: 0.01,
                                                         shininess: 1.5
                                                     });
        meshMaterial.shading = THREE.FlatShading;
        var mesh = new THREE.Mesh( bufferGeometry, meshMaterial );

        valterGroup.add(trunkGroup);
        trunkGroup.position.y = 0.735 - baseShiftY;

        trunkGroup.add(mesh);
    } );

    //-------------------------------------------------bodyGroup
    //body.json
    loader.load( "qrc:/json_models/resources/valter_model_json/body.json", function ( geometry, materials ) {
        geometry.computeVertexNormals();
        var bufferGeometry = new THREE.BufferGeometry();
        bufferGeometry.fromGeometry(geometry);
        var texture = THREE.ImageUtils.loadTexture("qrc:/textures/resources/valter_model_json/textures/aluminum.jpg");
        texture.minFilter = THREE.NearestFilter;
        var meshMaterial = new THREE.MeshPhongMaterial({
                                                         map: texture,
                                                         bumpMap: texture,
                                                         bumpScale: 0.002,
                                                         shininess: 1.5
                                                     });
        meshMaterial.shading = THREE.FlatShading;
        var mesh = new THREE.Mesh( bufferGeometry, meshMaterial );

        trunkGroup.add(bodyGroup);
        bodyGroup.position.y = 0.4;

        bodyGroup.add(mesh);

        //!!!!!!!!!!!!!! SET TO LAST LOADED MESH
        meshesLoaded = true;
    } );
}
