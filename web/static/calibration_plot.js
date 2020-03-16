/*============= Creating a canvas ======================*/
var canvas = document.getElementById('canvas');
gl = canvas.getContext('experimental-webgl');

/*========== Defining and storing the geometry ==========*/

var sphere_verties = [];

// verticies and triangles for an iscohedron
var t = (1 + Math.sqrt(5))/ 2; // golden ratio
var v = [[0, 1, t], [0, 1, -t], [0, -1, t], [0, -1, -t],
         [1, t, 0], [1, -t, 0], [-1, t, 0], [-1, -t, 0],
         [t, 0, 1], [t, 0, -1], [-t, 0, 1], [-t, 0, -1]];

var d = Math.sqrt(1+t*t);
for(var i=0; i<v.length; i++)
    for(var j=0; j<v.length; j++)
        v[i][j] /= d;

var triangles = [[0, 2, 8],  [0, 2, 10], [1, 3, 9],   [1, 3, 11],
                 [4, 6, 0],  [4, 6, 1],  [5, 7, 2],   [5, 7, 3],
                 [8, 9, 4],  [8, 9, 5],  [10, 11, 6], [10, 11, 7],
                 [0, 4, 8],  [1, 6, 11], [5, 9, 3],   [7, 11, 3],
                 [0, 10, 6], [1, 9, 4],  [2, 8, 5],   [2, 10, 7]];

function tesselate(v0, v1, v2, n) {
    function normalize(x, y, z) {
        d = Math.sqrt(x*x + y*y + z*z);
        return [x/d, y/d, z/d];
    }
    function line(v0, v1) {
        var c = 4;
        for(var i=0; i<c; i++) {
            var d0 = i/c;
            var d1 = (i+1)/c;
            var va = normalize(v0[0]*(1-d0) + v1[0]*d0,
                               v0[1]*(1-d0) + v1[1]*d0,
                               v0[2]*(1-d0) + v1[2]*d0);
            var vb = normalize(v0[0]*(1-d1) + v1[0]*d1,
                               v0[1]*(1-d1) + v1[1]*d1,
                               v0[2]*(1-d1) + v1[2]*d1);
            sphere_verties.push(va[0], va[1], va[2]);
            sphere_verties.push(vb[0], vb[1], vb[2]);
        }
    }
    if(n == 0) {
        line(v0, v1);
        line(v1, v2);
        line(v2, v0);
    } else {
        var va = normalize((v0[0]+v1[0])/2, (v0[1]+v1[1])/2, (v0[2]+v1[2])/2);
        var vb = normalize((v1[0]+v2[0])/2, (v1[1]+v2[1])/2, (v1[2]+v2[2])/2);
        var vc = normalize((v2[0]+v0[0])/2, (v2[1]+v0[1])/2, (v2[2]+v0[2])/2);

        tesselate(v0, vc, va, n-1);
        tesselate(va, vb, v1, n-1);
        tesselate(v2, vb, vc, n-1);
        tesselate(va, vb, vc, n-1);
    }
    
}

// tesselate an iscohedron a few times to get a pretty nice sphere
for(var t=0; t<triangles.length; t++) {
    triangle = triangles[t];
    v0 = v[triangle[0]];
    v1 = v[triangle[1]];
    v2 = v[triangle[2]];
    tesselate(v0, v1, v2, 1);
}

/*
  var lats = 16, lons = 32;
  for(var lat=1; lat<lats; lat++) {
  var flat = Math.PI*(lat/(lats-1)-.5);
  for(var lon=0; lon<lons; lon++) {
  var flon = 2*Math.PI*lon/(lons-1);
  sphere_verties.push( Math.cos(flat) * Math.cos(flon) );
  sphere_verties.push( Math.cos(flat) * Math.sin(flon) );
  sphere_verties.push( Math.sin(flat) );
  }
  }

  for(var lon=1; lon<lons/2; lon++) {
  var flon = 2*Math.PI*lon/(lons-1);
  for(var lat=0; lat<=lats*2; lat++) {
  var flat = Math.PI*(lat/(lats-1)+.5);
  sphere_verties.push( Math.cos(flat) * Math.cos(flon));
  sphere_verties.push( Math.cos(flat) * Math.sin(flon));
  sphere_verties.push( Math.sin(flat) );
  }
  }
*/
// Create and store data into vertex buffer
var sphere_vertex_buffer = gl.createBuffer ();
gl.bindBuffer(gl.ARRAY_BUFFER, sphere_vertex_buffer);
gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(sphere_verties), gl.STATIC_DRAW);


var compass_history = []
var compass_history_buffer = gl.createBuffer ();
var compass_calibration = [0, 0, 0, 30, 0];
var compass_sigmapoints_buffer = gl.createBuffer ();
var compass_sigmapoints = []
var compass_points_buffer = gl.createBuffer ();
var compass_points = []

/*=================== SHADERS =================== */

var vertCode = 'attribute vec3 position;'+
    'uniform mat4 Pmatrix;'+
    'uniform mat4 Vmatrix;'+
    'uniform mat4 Mmatrix;'+
    'void main(void) { '+//pre-built function
    'gl_Position = Pmatrix*Vmatrix*Mmatrix*vec4(position, 1.);'+
    'gl_PointSize = 3.0;'+
    '}';

var fragCode = 'precision mediump float;'+
    'uniform vec3 vColor;'+
    'void main(void) {'+
        'gl_FragColor = vec4(vColor, 1.);'+
    '}';

var vertShader = gl.createShader(gl.VERTEX_SHADER);
gl.shaderSource(vertShader, vertCode);
gl.compileShader(vertShader);

var fragShader = gl.createShader(gl.FRAGMENT_SHADER);
gl.shaderSource(fragShader, fragCode);
gl.compileShader(fragShader);

var shaderprogram = gl.createProgram();
gl.attachShader(shaderprogram, vertShader);
gl.attachShader(shaderprogram, fragShader);
gl.linkProgram(shaderprogram);

/*======== Associating attributes to vertex shader =====*/
var _Pmatrix = gl.getUniformLocation(shaderprogram, "Pmatrix");
var _Vmatrix = gl.getUniformLocation(shaderprogram, "Vmatrix");
var _Mmatrix = gl.getUniformLocation(shaderprogram, "Mmatrix");

var _color = gl.getUniformLocation(shaderprogram, "vColor");

var _position = gl.getAttribLocation(shaderprogram, "position");
gl.enableVertexAttribArray(_position);

gl.useProgram(shaderprogram);

/*==================== MATRIX ====================== */

function get_projection(angle, a, zMin, zMax) {
    var ang = Math.tan((angle*.5)*Math.PI/180);//angle*.5
    return [
        0.5/ang, 0 , 0, 0,
        0, 0.5*a/ang, 0, 0,
        0, 0, -(zMax+zMin)/(zMax-zMin), -1,
        0, 0, (-2*zMax*zMin)/(zMax-zMin), 0 
    ];
}

var proj_matrix = get_projection(40, canvas.width/canvas.height, 1, 100);
var mo_matrix = [ 1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1 ];
var view_matrix = [ 1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1 ];

view_matrix[14] = view_matrix[14]-3;

/*================= Mouse events ======================*/

var AMORTIZATION = 0.95;
var drag = false;
var old_x, old_y;
var dX = 0, dY = 0;

var mouseDown = function(e) {
    drag = true;
    old_x = e.pageX, old_y = e.pageY;
    e.preventDefault();
    return false;
};

var mouseUp = function(e){
    drag = false;
};

var mouseMove = function(e) {
    if (!drag) return false;
    dX = (e.pageX-old_x)*2*Math.PI/canvas.width,
    dY = (e.pageY-old_y)*2*Math.PI/canvas.height;
    THETA+= dX;
    PHI+=dY;
    old_x = e.pageX, old_y = e.pageY;
    e.preventDefault();
};

canvas.addEventListener("mousedown", mouseDown, false);
canvas.addEventListener("mouseup", mouseUp, false);
canvas.addEventListener("mouseout", mouseUp, false);
canvas.addEventListener("mousemove", mouseMove, false);

/*=========================rotation================*/

function rotateX(m, angle) {
    var c = Math.cos(angle);
    var s = Math.sin(angle);
    var mv1 = m[1], mv5 = m[5], mv9 = m[9];

    m[1] = m[1]*c-m[2]*s;
    m[5] = m[5]*c-m[6]*s;
    m[9] = m[9]*c-m[10]*s;

    m[2] = m[2]*c+mv1*s;
    m[6] = m[6]*c+mv5*s;
    m[10] = m[10]*c+mv9*s;
}

function rotateY(m, angle) {
    var c = Math.cos(angle);
    var s = Math.sin(angle);
    var mv0 = m[0], mv4 = m[4], mv8 = m[8];

    m[0] = c*m[0]+s*m[2];
    m[4] = c*m[4]+s*m[6];
    m[8] = c*m[8]+s*m[10];

    m[2] = c*m[2]-s*mv0;
    m[6] = c*m[6]-s*mv4;
    m[10] = c*m[10]-s*mv8;
}

/*=================== Drawing =================== */

var THETA = 0,
    PHI = 0;
var time_old = 0;

var animate = function(time) {
    var dt = time-time_old;

    if (!drag) {
        dX *= AMORTIZATION, dY*=AMORTIZATION;
        THETA+=dX, PHI+=dY;
    }


    time_old = time; 
    gl.enable(gl.DEPTH_TEST);

    // gl.depthFunc(gl.LEQUAL);

    gl.clearColor(0, 0, 0, 1);
    gl.clearDepth(2);
    gl.viewport(0.0, 0.0, canvas.width, canvas.height);
    gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);


    //set model matrix to I4

    mo_matrix[0] = 1, mo_matrix[1] = 0, mo_matrix[2] = 0,
    mo_matrix[3] = 0,

    mo_matrix[4] = 0, mo_matrix[5] = 1, mo_matrix[6] = 0,
    mo_matrix[7] = 0,

    mo_matrix[8] = 0, mo_matrix[9] = 0, mo_matrix[10] = 1,
    mo_matrix[11] = 0,

    mo_matrix[12] = 0, mo_matrix[13] = 0, mo_matrix[14] = 0,
    mo_matrix[15] = 1;

    rotateY(mo_matrix, THETA);
    rotateX(mo_matrix, PHI);

    
    gl.uniformMatrix4fv(_Pmatrix, false, proj_matrix);
    gl.uniformMatrix4fv(_Vmatrix, false, view_matrix);
    gl.uniformMatrix4fv(_Mmatrix, false, mo_matrix);

    gl.uniform3f(_color, 0, 0, .8);
    
    gl.bindBuffer(gl.ARRAY_BUFFER, sphere_vertex_buffer);
    gl.vertexAttribPointer(_position, 3, gl.FLOAT, false,0,0);
    gl.drawArrays(gl.LINES, 0, sphere_verties.length/3);

    gl.uniform3f(_color, 1, 1, 0);
    
    gl.bindBuffer(gl.ARRAY_BUFFER, compass_sigmapoints_buffer);
    gl.vertexAttribPointer(_position, 3, gl.FLOAT, false,0,0);
    gl.drawArrays(gl.POINTS, 0, compass_sigmapoints.length);


    gl.uniform3f(_color, 0, 1, 1);
    
    gl.bindBuffer(gl.ARRAY_BUFFER, compass_points_buffer);
    gl.vertexAttribPointer(_position, 3, gl.FLOAT, false,0,0);
    gl.drawArrays(gl.POINTS, 0, compass_points.length);


    if(compass_history.length >= 5) {
        gl.uniform3f(_color, 0, 1, 0);
        gl.bindBuffer(gl.ARRAY_BUFFER, compass_history_buffer);
        gl.vertexAttribPointer(_position, 3, gl.FLOAT, false,0,0);
        gl.drawArrays(gl.POINTS, 0, compass_history.length-5);

        gl.uniform3f(_color, 1, 0, 0);
        gl.drawArrays(gl.POINTS, compass_history.length-5, 5);
    }
    
    window.requestAnimationFrame(animate);
}
animate(0);

function convert_points(data, calibration) {
    points = []
    var x = calibration[0];
    var y = calibration[1];
    var z = calibration[2];
    var s = calibration[3];
    for(var i=0; i<data.length; i++) {
        var c = data[i];
        points.push((c[0]-x)/s);
        points.push((c[1]-y)/s);
        points.push((c[2]-z)/s);
    }
    return points;
}


$(document).ready(function() {
    namespace = '';
    $('#connection').text("N/A");
    $('#compass_calibration').text("N/A");


    // Connect to the Socket.IO server.
    var port = location.port;
    port = pypilot_web_port;
    var socket = io.connect(location.protocol + '//' + document.domain + ':' + port + namespace);

    function pypilot_watch(name, period=true) {
        socket.emit('pypilot', 'watch={"' + name + '":' + JSON.stringify(period) + '}')
    }
    
    socket.on('pypilot_values', function(msg) {
        //var list_values = JSON.parse(msg);
        $('#connection').text('Connected');

        pypilot_watch('imu.compass', .25);
        pypilot_watch('imu.compass.calibration');
        pypilot_watch('imu.compass.calibration.sigmapoints')
        pypilot_watch('imu.compass.calibration.points')
    });

    socket.on('pypilot_disconnect', function() {
        $('#connection').text('Disconnected')
    });
    
    socket.on('pypilot', function(msg) {
        data = JSON.parse(msg);

        if('imu.compass.calibration' in data)
            compass_calibration = data['imu.compass.calibration'][0];
            $('#compass_calibration').text(data['imu.compass.calibration']);

        if('imu.compass' in data) {
            compass_history.push(data['imu.compass'])
            if(compass_history.length > 40)
                compass_history.shift();
            var compass_history_points = convert_points(compass_history, compass_calibration);
            gl.bindBuffer(gl.ARRAY_BUFFER, compass_history_buffer);
            gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(compass_history_points), gl.STATIC_DRAW);
        }

        if('imu.compass.calibration.sigmapoints' in data) {
            compass_sigmapoints = data['imu.compass.calibration.sigmapoints'];
            var compass_sigmapoints_points = convert_points(compass_sigmapoints, compass_calibration);
            gl.bindBuffer(gl.ARRAY_BUFFER, compass_sigmapoints_buffer);
            gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(compass_sigmapoints_points), gl.STATIC_DRAW);   
        }

        if('imu.compass.calibration.points' in data) {
            compass_points = data['imu.compass.calibration.points'];
            var compass_points_points = convert_points(compass_points, compass_calibration);
            gl.bindBuffer(gl.ARRAY_BUFFER, compass_points_buffer);
            gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(compass_points_points), gl.STATIC_DRAW);   
        }
    });    
});
