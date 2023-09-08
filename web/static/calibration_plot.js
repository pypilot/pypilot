/*
#   Copyright (C) 2023 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  
*/

/*============= Creating a canvas ======================*/
var canvas = document.getElementById('canvas');
gl = canvas.getContext('experimental-webgl');

/*========== Defining and storing the geometry ==========*/

var sphere_vertices = [];

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
            sphere_vertices.push(va[0], va[1], va[2]);
            sphere_vertices.push(vb[0], vb[1], vb[2]);
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
  sphere_vertices.push( Math.cos(flat) * Math.cos(flon) );
  sphere_vertices.push( Math.cos(flat) * Math.sin(flon) );
  sphere_vertices.push( Math.sin(flat) );
  }
  }

  for(var lon=1; lon<lons/2; lon++) {
  var flon = 2*Math.PI*lon/(lons-1);
  for(var lat=0; lat<=lats*2; lat++) {
  var flat = Math.PI*(lat/(lats-1)+.5);
  sphere_vertices.push( Math.cos(flat) * Math.cos(flon));
  sphere_vertices.push( Math.cos(flat) * Math.sin(flon));
  sphere_vertices.push( Math.sin(flat) );
  }
  }
*/
// Create and store data into vertex buffer
var sphere_vertex_buffer = gl.createBuffer ();
gl.bindBuffer(gl.ARRAY_BUFFER, sphere_vertex_buffer);
gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(sphere_vertices), gl.STATIC_DRAW);


var point_history = []
var point_history_buffer = gl.createBuffer ();
var accel_calibration = [0, 0, 0, 1];
var compass_calibration = [0, 0, 0, 30, 0];
var sigmapoints_buffer = gl.createBuffer ();
var sigmapoints = []
var points_buffer = gl.createBuffer ();
var points = []

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
    gl.drawArrays(gl.LINES, 0, sphere_vertices.length/3);

    gl.uniform3f(_color, 1, 1, 0);
    
    gl.bindBuffer(gl.ARRAY_BUFFER, sigmapoints_buffer);
    gl.vertexAttribPointer(_position, 3, gl.FLOAT, false,0,0);
    gl.drawArrays(gl.POINTS, 0, sigmapoints.length);


    gl.uniform3f(_color, 0, 1, 1);
    
    gl.bindBuffer(gl.ARRAY_BUFFER, points_buffer);
    gl.vertexAttribPointer(_position, 3, gl.FLOAT, false,0,0);
    gl.drawArrays(gl.POINTS, 0, points.length);


    if(point_history.length >= 5) {
        gl.uniform3f(_color, 0, 1, 0);
        gl.bindBuffer(gl.ARRAY_BUFFER, point_history_buffer);
        gl.vertexAttribPointer(_position, 3, gl.FLOAT, false,0,0);
        gl.drawArrays(gl.POINTS, 0, point_history.length-5);

        gl.uniform3f(_color, 1, 0, 0);
        gl.drawArrays(gl.POINTS, point_history.length-5, 5);
    }
    
    window.requestAnimationFrame(animate);
}
animate(0);

function convert_points(data, calibration) {
    p = []
    var x = calibration[0];
    var y = calibration[1];
    var z = calibration[2];
    var s = calibration[3];
    for(var i=0; i<data.length; i++) {
        var c = data[i];
        p.push((c[0]-x)/s);
        p.push((c[1]-y)/s);
        p.push((c[2]-z)/s);
    }
    return p;
}

var current_plot = false;
var plots = ['accel', 'compass'];

$(document).ready(function() {
    namespace = '';
    $('#connection').text("N/A");
    $('#calibration').text("N/A");

    // Connect to the Socket.IO server.
    var port = location.port;
    port = pypilot_web_port;
    var socket = io.connect(location.protocol + '//' + document.domain + ':' + port + namespace);

    function pypilot_watch(name, period=true) {
        socket.emit('pypilot', 'watch={"' + name + '":' + JSON.stringify(period) + '}')
    }
    
    socket.on('pypilot_values', function(msg) {
        $('#connection').text('Connected');

        for(s of ['error', 'warning'])
            pypilot_watch('imu.' + s)
        
        watches = ['', '.age', '.log', '.warning'].map(function (e) { return '.calibration'+e; });
        for(var watch in watches)
            for(var plot in plots)
                pypilot_watch('imu.' + plots[plot] + watches[watch]);
        current_plot = false;
        switch_plot();
    });

    socket.on('pypilot_disconnect', function() {
        $('#connection').text('Disconnected')
    });

    function switch_plot() {
        point_history = [];

        watches = ['', '.calibration.sigmapoints', '.calibration.points'];
        period = .25;
        if(current_plot)
            for(var watch in watches)
                pypilot_watch('imu.' + current_plot + watches[watch], false);

        if(document.getElementById('accel').checked)
            current_plot = 'accel';
        else
            current_plot = 'compass';

        for(var watch in watches)
            pypilot_watch('imu.' + current_plot + watches[watch], period);
    }
    
    $('#accel').click(switch_plot);
    $('#compass').click(switch_plot);

    var calibration_log = {'accel': [], 'compass': []}
    socket.on('pypilot', function(msg) {
        data = JSON.parse(msg);
        for(s of ['error', 'warning']) {
            name = 'imu.'+s;
            if(name in data)
                $('#imu_'+s).text(data[name])
        }

        for(s of ['accel', 'compass'])
            for(n of ['', 'age', 'log']) {
                name = 'imu.' + s + '.calibration';
                id = '#'+s+'_calibration'
                if(n) {
                    name += '.' + n
                    id += '_' + n
                }
                
                if(name in data) {
                    value = data[name];
                    if(!n) {
                        value = value[0];
                        if(s == 'accel')
                            accel_calibration = value;
                        else
                            compass_calibration = value;
                    }
                    if(n == 'log') {
                        calibration_log[s].push(value);
                        if(calibration_log[s].length > 4)
                            calibration_log[s].shift();
                        value = '';
                        for(line of calibration_log[s])
                            value += line + '\n';
                    }
                    $(id).text(value);
                }
            }

        if(current_plot == 'accel')
            calibration = accel_calibration;
        else
            calibration = compass_calibration;

        n = 'imu.' + current_plot
        if(n in data) {
            point_history.push(data[n])
            if(point_history.length > 40)
                point_history.shift();
            var point_history_points = convert_points(point_history, calibration);
            gl.bindBuffer(gl.ARRAY_BUFFER, point_history_buffer);
            gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(point_history_points), gl.STATIC_DRAW);
        }

        n = 'imu.' + current_plot + '.calibration.sigmapoints'
        if(n in data) {
            sigmapoints = data[n];
            var sigmapoints_points = convert_points(sigmapoints, calibration);
            gl.bindBuffer(gl.ARRAY_BUFFER, sigmapoints_buffer);
            gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(sigmapoints_points), gl.STATIC_DRAW);   
        }

        n = 'imu.' + current_plot + '.calibration.points'
        if(n in data) {
            points = data[n];
            var points_points = convert_points(points, calibration);
            gl.bindBuffer(gl.ARRAY_BUFFER, points_buffer);
            gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(points_points), gl.STATIC_DRAW);   
        }
    });
});
