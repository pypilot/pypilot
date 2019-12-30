/*
#
#   Copyright (C) 2019 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  
*/

$(document).ready(function() {
    namespace = '';
    $('#code').text("N/A");
    // Connect to the Socket.IO server.
    var port = location.port;
    port = web_port;
    var socket = io.connect(location.protocol + '//' + document.domain + ':' + port + namespace);
    //socket.emit('signalk', JSON.stringify({'name': name, 'method': 'get'}));
    // Event handler for new connections.
    socket.on('connect', function(msg) {
        $('#connection').text('Connected')
    });

    socket.on('disconnect', function() {
        $('#connection').text('Disconnected')
    });

    socket.on('pypilot', function(log) {
        $('#pypilot').text(log);
    });

    socket.on('code', function(code) {
        $('#code').text(code);
    });

    socket.on('action', function(action) {
        $('#action').text(action);
    });
    
    socket.on('action_codes', function(codes) {
        $('#action'+codes['name']+'codes').text(codes['codes'])
    });
             
    $('#engage').click(function(event) {
        socket.emit('codes', 'engage');
    });

    $('#disengage').click(function(event) {
        socket.emit('codes', 'disengage');
    });
    
    $('#m1').click(function(event) {
        socket.emit('codes', '1');
    });
    
    $('#m2').click(function(event) {
        socket.emit('codes', '2');
    });
    
    $('#m10').click(function(event) {
        socket.emit('codes', '10');
    });
    
    
    $('#m-1').click(function(event) {
        socket.emit('codes', '-1');
    });
    
    $('#m-2').click(function(event) {
        socket.emit('codes', '-2');
    });
    
    $('#m-10').click(function(event) {
        socket.emit('codes', '-10');
    });

    $('#compassmode').click(function(event) {
        socket.emit('codes', 'compassmode');
    });
    
    $('#gpsmode').click(function(event) {
        socket.emit('codes', 'gpsmode');
    });

    $('#windmode').click(function(event) {
        socket.emit('codes', 'windmode');
    });

    // Interval function that tests message latency by sending a "ping"
    var ping_pong_times = [];
    var start_time;
    window.setInterval(function() {
        start_time = (new Date).getTime();
        socket.emit('ping');
        //        $('#log').append("ping" + "<br>");
        $('#ping-pong').text('pn');
    }, 5000);
    
    // Handler for the "pong" message. When the pong is received, the
    socket.on('pong', function() {
        var latency = (new Date).getTime() - start_time;
        ping_pong_times.push(latency);
        ping_pong_times = ping_pong_times.slice(-30); // keep last 30 samples
        var sum = 0;
        for (var i = 0; i < ping_pong_times.length; i++)
            sum += ping_pong_times[i];
        $('#ping-pong').text(Math.round(10 * sum / ping_pong_times.length) / 10);
    });

    function window_resize() {
        var w = $(window).width();
        $(".font-resizable").each(function(i, obj) {
            if (w < 600)
                $(this).css('font-size', w/18+"px")
            else
                $(this).css('font-size', w/35+"px")
        });
        $(".font-resizable1").each(function(i, obj) {
            $(this).css('font-size', w/12+"px")
        });
        $(".font-resizable2").each(function(i, obj) {
            $(this).css('font-size', w/30+"px")
        });
        $(".font-resizable3").each(function(i, obj) {
            $(this).css('font-size', w/50+"px")
        });
        $(".button-resizable").each(function(i, obj) {
            $(this).css('width', w/5+"px")
            $(this).css('height', w/6+"px")
        });
        $(".button-resizable1").each(function(i, obj) {
            $(this).css('width', w/5+"px")
            $(this).css('height', w/18+"px")
        });
        $(".button-resizable2").each(function(i, obj) {
            $(this).css('width', w/8+"px")
            $(this).css('height', w/18+"px")
        });
        $(".toggle-button-selected button").each(function(i, obj) {
            $(this).css('left', w/12+"px")
        });
    }
    $(window).resize(window_resize);
    window_resize();
});
