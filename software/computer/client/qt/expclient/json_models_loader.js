
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
        var meshMaterial = new THREE.MeshPhongMaterial({color: "#E6E9EB", specular: "#555555", shininess: 30});
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

        bodyGroupHelperMeshX = new THREE.Mesh();
        bodyGroupHelperMeshX.position.x = 1.0;
        bodyGroup.add(bodyGroupHelperMeshX);

        bodyGroupHelperMeshZ = new THREE.Mesh();
        bodyGroupHelperMeshZ.position.z = 1.0;
        bodyGroup.add(bodyGroupHelperMeshZ);

        bodyGroupHelperMeshCenter = new THREE.Mesh();
        bodyGroup.add(bodyGroupHelperMeshCenter);

        bodyGroup.add(mesh);
    } );

    //-------------------------------------------------leftShoulderGroup
    //body.shoulder.left.json
    loader.load( "qrc:/json_models/resources/valter_model_json/body.shoulder.left.json", function ( geometry, materials ) {
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

        mesh.rotation.y = degToRad(180);

        bodyGroup.add(leftShoulderGroup);

        leftShoulderGroup.position.y = 0.320;
        leftShoulderGroup.position.x = 0.108;
        leftShoulderGroup.position.z = -0.09;

        leftShoulderGroup.add(mesh);

    } );

    //body.shoulder.right.json
    loader.load( "qrc:/json_models/resources/valter_model_json/body.shoulder.right.json", function ( geometry, materials ) {
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

        mesh.rotation.y = degToRad(180);

        bodyGroup.add(rightShoulderGroup);

        rightShoulderGroup.position.y = 0.320;
        rightShoulderGroup.position.x = 0.108;
        rightShoulderGroup.position.z = 0.09;

        rightShoulderGroup.add(mesh);
    } );

    //-------------------------------------------------leftShoulderLinkGroup
    //shoulder.link.json
    loader.load( "qrc:/json_models/resources/valter_model_json/shoulder.link.json", function ( geometry, materials ) {
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

        mesh.rotation.z = degToRad(90);

        leftShoulderGroup.add(leftShoulderLinkGroup);

        leftShoulderLinkGroup.position.y = 0.001;
        leftShoulderLinkGroup.position.x = -0.1085;
        leftShoulderLinkGroup.position.z = -0.183;

        leftShoulderLinkGroup.add(mesh);


        //!!!!!!!!!!!!!! SET TO LAST LOADED MESH
        meshesLoaded = true;
    } );

    //-------------------------------------------------rightShoulderLinkGroup
    //shoulder.link.json
    loader.load( "qrc:/json_models/resources/valter_model_json/shoulder.link.json", function ( geometry, materials ) {
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

        mesh.rotation.z = degToRad(90);
        mesh.rotation.y = degToRad(180);

        rightShoulderGroup.add(rightShoulderLinkGroup);

        rightShoulderLinkGroup.position.y = 0.001;
        rightShoulderLinkGroup.position.x = -0.1085;
        rightShoulderLinkGroup.position.z = 0.183;

        rightShoulderLinkGroup.add(mesh);
    } );

    //-------------------------------------------------leftArmLinkGroup
    //arm.link.json
    loader.load( "qrc:/json_models/resources/valter_model_json/arm.link.json", function ( geometry, materials ) {
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

        leftShoulderLinkGroup.add(leftArmLinkGroup);

        leftArmLinkGroup.position.y = 0.0;
        leftArmLinkGroup.position.x = 0.0;
        leftArmLinkGroup.position.z = 0.0;

        leftArmLinkGroup.add(mesh);
    } );

    //-------------------------------------------------rightArmLinkGroup
    //arm.link.json
    loader.load( "qrc:/json_models/resources/valter_model_json/arm.link.json", function ( geometry, materials ) {
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

        rightShoulderLinkGroup.add(rightArmLinkGroup);

        rightArmLinkGroup.position.y = 0.0;
        rightArmLinkGroup.position.x = 0.0;
        rightArmLinkGroup.position.z = 0.0;

        rightArmLinkGroup.add(mesh);
    } );

    //-------------------------------------------------leftElbowGroup
    //elbow.json
    loader.load( "qrc:/json_models/resources/valter_model_json/elbow.json", function ( geometry, materials ) {
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

        leftArmLinkGroup.add(leftElbowGroup);

        leftElbowGroup.position.y = -0.4;
        leftElbowGroup.position.x = 0.0;
        leftElbowGroup.position.z = 0.0;

        leftElbowGroup.add(mesh);
    } );

    //-------------------------------------------------rightElbowGroup
    //elbow.json
    loader.load( "qrc:/json_models/resources/valter_model_json/elbow.json", function ( geometry, materials ) {
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

        rightArmLinkGroup.add(rightElbowGroup);

        rightElbowGroup.position.y = -0.4;
        rightElbowGroup.position.x = 0.0;
        rightElbowGroup.position.z = 0.0;

        rightElbowGroup.add(mesh);


        //!!!!!!!!!!!!!! SET TO LAST LOADED MESH
        meshesLoaded = true;
    } );

    //-------------------------------------------------leftForearmYawGroup
    //forearm.p1.json
    loader.load( "qrc:/json_models/resources/valter_model_json/forearm.p1.json", function ( geometry, materials ) {
        geometry.computeVertexNormals();
        var bufferGeometry = new THREE.BufferGeometry();
        bufferGeometry.fromGeometry(geometry);
        var meshMaterial = new THREE.MeshPhongMaterial({color: "#1D1E1F", specular: "#555555", shininess: 25});
        meshMaterial.shading = THREE.FlatShading;
        var mesh = new THREE.Mesh( bufferGeometry, meshMaterial );

        leftElbowGroup.add(leftForearmYawGroup);

        leftForearmYawGroup.position.y = -0.036;
        leftForearmYawGroup.position.x = 0.0;
        leftForearmYawGroup.position.z = 0.0;

        leftForearmYawGroup.add(mesh);
    } );

    //-------------------------------------------------rightForearmYawGroup
    //forearm.p1.json
    loader.load( "qrc:/json_models/resources/valter_model_json/forearm.p1.json", function ( geometry, materials ) {
        geometry.computeVertexNormals();
        var bufferGeometry = new THREE.BufferGeometry();
        bufferGeometry.fromGeometry(geometry);
        var meshMaterial = new THREE.MeshPhongMaterial({color: "#1D1E1F", specular: "#555555", shininess: 25});
        meshMaterial.shading = THREE.FlatShading;
        var mesh = new THREE.Mesh( bufferGeometry, meshMaterial );

        rightElbowGroup.add(rightForearmYawGroup);

        rightForearmYawGroup.position.y = -0.036;
        rightForearmYawGroup.position.x = 0.0;
        rightForearmYawGroup.position.z = 0.0;

        rightForearmYawGroup.add(mesh);
    } );

    //-------------------------------------------------leftForearmRollGroup
    //forearm.json
    loader.load( "qrc:/json_models/resources/valter_model_json/forearm.json", function ( geometry, materials ) {
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

        mesh.rotation.z = degToRad(180);
        mesh.rotation.x = degToRad(90);

        leftForearmYawGroup.add(leftForearmRollGroup);

        leftForearmRollGroup.position.y = -0.055;
        leftForearmRollGroup.position.x = 0.04;
        leftForearmRollGroup.position.z = 0.0;

        leftForearmRollGroup.add(mesh);
    } );

    //-------------------------------------------------rightForearmRollGroup
    //forearm.json
    loader.load( "qrc:/json_models/resources/valter_model_json/forearm.json", function ( geometry, materials ) {
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

        mesh.rotation.z = degToRad(180);
        mesh.rotation.x = degToRad(90);

        rightForearmYawGroup.add(rightForearmRollGroup);

        rightForearmRollGroup.position.y = -0.055;
        rightForearmRollGroup.position.x = 0.04;
        rightForearmRollGroup.position.z = 0.0;

        rightForearmRollGroup.add(mesh);
    } );

    //-------------------------------------------------leftPalmTiltGroup
    //palm.lift.fixture.p1.json
    loader.load( "qrc:/json_models/resources/valter_model_json/palm.lift.fixture.p1.json", function ( geometry, materials ) {
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

        mesh.rotation.x = degToRad(90);
        mesh.rotation.z = degToRad(90);
        mesh.rotation.y = 0.0;

        leftForearmRollGroup.add(leftPalmTiltGroup);

        leftPalmTiltGroup.position.y = -0.005;
        leftPalmTiltGroup.position.x = 0.234;
        leftPalmTiltGroup.position.z = 0.0;

        leftPalmTiltGroup.add(mesh);
    } );

    //-------------------------------------------------rightPalmTiltGroup
    //palm.lift.fixture.p1.json
    loader.load( "qrc:/json_models/resources/valter_model_json/palm.lift.fixture.p1.json", function ( geometry, materials ) {
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

        mesh.rotation.x = degToRad(90);
        mesh.rotation.z = degToRad(90);
        mesh.rotation.y = 0.0;

        rightForearmRollGroup.add(rightPalmTiltGroup);

        rightPalmTiltGroup.position.y = -0.005;
        rightPalmTiltGroup.position.x = 0.234;
        rightPalmTiltGroup.position.z = 0.0;

        rightPalmTiltGroup.add(mesh);
    } );

    //-------------------------------------------------leftPalmGroup
    //palm.json
    loader.load( "qrc:/json_models/resources/valter_model_json/palm.json", function ( geometry, materials ) {
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

        mesh.rotation.x = degToRad(90);
        mesh.rotation.z = degToRad(-90);

        leftPalmTiltGroup.add(leftPalmGroup);

        leftPalmGroup.position.y = 0.012;
        leftPalmGroup.position.x = 0.016;
        leftPalmGroup.position.z = 0.0;

        leftPalmGroup.add(mesh);
    } );

    //-------------------------------------------------rightPalmGroup
    //palm.json
    loader.load( "qrc:/json_models/resources/valter_model_json/palm.json", function ( geometry, materials ) {
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

        mesh.rotation.x = degToRad(90);
        mesh.rotation.z = degToRad(-90);

        rightPalmTiltGroup.add(rightPalmGroup);

        rightPalmGroup.position.y = 0.012;
        rightPalmGroup.position.x = 0.016;
        rightPalmGroup.position.z = 0.0;

        rightPalmGroup.add(mesh);
    } );

    //-------------------------------------------------finger groups
    //finger.json
    loader.load( "qrc:/json_models/resources/valter_model_json/finger.json", function ( geometry, materials ) {
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


        var leftFinger0mesh = new THREE.Mesh( bufferGeometry, meshMaterial );
        var leftFinger1mesh = new THREE.Mesh( bufferGeometry, meshMaterial );
        var leftFinger2mesh = new THREE.Mesh( bufferGeometry, meshMaterial );
        var leftFinger3mesh = new THREE.Mesh( bufferGeometry, meshMaterial );
        var leftFinger4mesh = new THREE.Mesh( bufferGeometry, meshMaterial );
        var leftFinger5mesh = new THREE.Mesh( bufferGeometry, meshMaterial );

        var rightFinger0mesh = new THREE.Mesh( bufferGeometry, meshMaterial );
        var rightFinger1mesh = new THREE.Mesh( bufferGeometry, meshMaterial );
        var rightFinger2mesh = new THREE.Mesh( bufferGeometry, meshMaterial );
        var rightFinger3mesh = new THREE.Mesh( bufferGeometry, meshMaterial );
        var rightFinger4mesh = new THREE.Mesh( bufferGeometry, meshMaterial );
        var rightFinger5mesh = new THREE.Mesh( bufferGeometry, meshMaterial );

        //left palm
        leftPalmGroup.add(leftFinger0Group);
        leftPalmGroup.add(leftFinger1Group);
        leftPalmGroup.add(leftFinger2Group);
        leftPalmGroup.add(leftFinger3Group);
        leftPalmGroup.add(leftFinger4Group);
        leftPalmGroup.add(leftFinger5Group);

        leftFinger0Group.position.y = 0.0005;
        leftFinger0Group.position.x = 0.0147;
        leftFinger0Group.position.z = -0.035;

        leftFinger1Group.position.y = 0.0015;
        leftFinger1Group.position.x = 0.0866;
        leftFinger1Group.position.z = -0.034;

        leftFinger2Group.position.y = 0.0015;
        leftFinger2Group.position.x = 0.0866;
        leftFinger2Group.position.z = -0.0115;

        leftFinger3Group.position.y = 0.0015;
        leftFinger3Group.position.x = 0.0866;
        leftFinger3Group.position.z = 0.011;

        leftFinger4Group.position.y = 0.0015;
        leftFinger4Group.position.x = 0.0866;
        leftFinger4Group.position.z = 0.0335;

        leftFinger5Group.position.y = 0.0005;
        leftFinger5Group.position.x = 0.0147;
        leftFinger5Group.position.z = 0.035;

        leftFinger0mesh.rotation.y = degToRad(-160);
        leftFinger0mesh.rotation.x = degToRad(-90);
        leftFinger0Group.add(leftFinger0mesh);

        leftFinger1mesh.rotation.y = degToRad(10);
        leftFinger1mesh.rotation.x = degToRad(90);
        leftFinger1Group.add(leftFinger1mesh);

        leftFinger2mesh.rotation.y = degToRad(10);
        leftFinger2mesh.rotation.x = degToRad(90);
        leftFinger2Group.add(leftFinger2mesh);

        leftFinger3mesh.rotation.y = degToRad(10);
        leftFinger3mesh.rotation.x = degToRad(90);
        leftFinger3Group.add(leftFinger3mesh);

        leftFinger4mesh.rotation.y = degToRad(10);
        leftFinger4mesh.rotation.x = degToRad(90);
        leftFinger4Group.add(leftFinger4mesh);

        leftFinger5mesh.rotation.y = degToRad(-160);
        leftFinger5mesh.rotation.x = degToRad(-90);
        leftFinger5Group.add(leftFinger5mesh);

        //right palm
        rightPalmGroup.add(rightFinger0Group);
        rightPalmGroup.add(rightFinger1Group);
        rightPalmGroup.add(rightFinger2Group);
        rightPalmGroup.add(rightFinger3Group);
        rightPalmGroup.add(rightFinger4Group);
        rightPalmGroup.add(rightFinger5Group);

        rightFinger0Group.position.y = 0.0005;
        rightFinger0Group.position.x = 0.0147;
        rightFinger0Group.position.z = -0.035;

        rightFinger1Group.position.y = 0.0015;
        rightFinger1Group.position.x = 0.0866;
        rightFinger1Group.position.z = -0.034;

        rightFinger2Group.position.y = 0.0015;
        rightFinger2Group.position.x = 0.0866;
        rightFinger2Group.position.z = -0.0115;

        rightFinger3Group.position.y = 0.0015;
        rightFinger3Group.position.x = 0.0866;
        rightFinger3Group.position.z = 0.011;

        rightFinger4Group.position.y = 0.0015;
        rightFinger4Group.position.x = 0.0866;
        rightFinger4Group.position.z = 0.0335;

        rightFinger5Group.position.y = 0.0005;
        rightFinger5Group.position.x = 0.0147;
        rightFinger5Group.position.z = 0.035;

        rightFinger0mesh.rotation.y = degToRad(-160);
        rightFinger0mesh.rotation.x = degToRad(-90);
        rightFinger0Group.add(rightFinger0mesh);

        rightFinger1mesh.rotation.y = degToRad(10);
        rightFinger1mesh.rotation.x = degToRad(90);
        rightFinger1Group.add(rightFinger1mesh);

        rightFinger2mesh.rotation.y = degToRad(10);
        rightFinger2mesh.rotation.x = degToRad(90);
        rightFinger2Group.add(rightFinger2mesh);

        rightFinger3mesh.rotation.y = degToRad(10);
        rightFinger3mesh.rotation.x = degToRad(90);
        rightFinger3Group.add(rightFinger3mesh);

        rightFinger4mesh.rotation.y = degToRad(10);
        rightFinger4mesh.rotation.x = degToRad(90);
        rightFinger4Group.add(rightFinger4mesh);

        rightFinger5mesh.rotation.y = degToRad(-160);
        rightFinger5mesh.rotation.x = degToRad(-90);
        rightFinger5Group.add(rightFinger5mesh);
    } );

    //-------------------------------------------------headYawGroup
    //head.p1.json
    loader.load( "qrc:/json_models/resources/valter_model_json/head.p1.json", function ( geometry, materials ) {
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

        mesh.rotation.x = degToRad(90);
        mesh.rotation.z = degToRad(90);

        bodyGroup.add(headYawGroup);
        headYawGroup.position.y = 0.42;

        headYawGroup.add(mesh);

        headYawGroupHelperMesh = new THREE.Mesh();
        headYawGroup.add(headYawGroupHelperMesh);

        headYawGroupXMesh = new THREE.Mesh();
        headYawGroupXMesh.position.x = 1.0;
        headYawGroup.add(headYawGroupXMesh);

        headYawGroupYMesh = new THREE.Mesh();
        headYawGroupYMesh.position.y = 0.5;
        headYawGroup.add(headYawGroupYMesh);

    } );

    //-------------------------------------------------headGroup
    //head.json
    loader.load( "qrc:/json_models/resources/valter_model_json/head.json", function ( geometry, materials ) {
        geometry.computeVertexNormals();
        var bufferGeometry = new THREE.BufferGeometry();
        bufferGeometry.fromGeometry(geometry);
        var texture = THREE.ImageUtils.loadTexture("qrc:/textures/resources/valter_model_json/textures/texture.jpg");
        texture.minFilter = THREE.NearestFilter;
        var meshMaterial = new THREE.MeshPhongMaterial({
                                                         map: texture,
                                                         bumpMap: texture,
                                                         bumpScale: 1.0,
                                                         shininess: 1.5
                                                     });
        meshMaterial.shading = THREE.FlatShading;
        var mesh = new THREE.Mesh( bufferGeometry, meshMaterial );

        mesh.rotation.x = degToRad(90);
        mesh.rotation.z = degToRad(90);

        headYawGroup.add(headGroup);
        headGroup.position.y = 0.05;
        headGroup.position.x = 0.0085;

        headGroup.add(mesh);
    } );

    //-------------------------------------------------headEndEffectorGroup
    //man.end.effector.json
    loader.load( "qrc:/json_models/resources/valter_model_json/man.end.effector.json", function ( geometry, materials ) {
        geometry.computeVertexNormals();
        var bufferGeometry = new THREE.SphereGeometry( 0.01, 12, 12 );

        headEndEffectorGroupVectorMesh = new THREE.Mesh( );
        var meshMaterial = new THREE.MeshPhongMaterial({color: "#FF0000", specular: "#555555", shininess: 25});
        headEndEffectorGroupVectorHelperMesh = new THREE.Mesh(bufferGeometry , meshMaterial);

        headGroup.add(headEndEffectorGroup);

        headEndEffectorGroup.position.y = 0.125;

        headEndEffectorGroupVectorHelperMesh.position.x = 1.0;

        headEndEffectorGroup.add(headEndEffectorGroupVectorMesh);
        headEndEffectorGroup.add(headEndEffectorGroupVectorHelperMesh);



        //!!!!!!!!!!!!!! SET TO THE LAST LOADED MESH
        meshesLoaded = true;
    } );


}
