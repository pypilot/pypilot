/*
#   Copyright (C) 2020 Sean D'Epagnier
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
    //socket.emit('pypilot', JSON.stringify({'name': name, 'method': 'get'}));
    // Event handler for new connections.
    socket.on('connect', function(msg) {
        $('#connection').text('connected')
    });

    socket.on('disconnect', function() {
        $('#connection').text('disconnected')
    });

    socket.on('status', function(msg) {
        $('#status').text(msg);
    });

    socket.on('baudrate', function(msg) {
        $('#baudrate').text(msg);
    });
    
    socket.on('key', function(key) {
        $('#key0').text(key);
        $('#key1').text(key);
    });

    socket.on('action', function(action) {
        $('#action0').text(action);
        $('#action1').text(action);
    });
    
    socket.on('action_keys', function(keys) {
        $('#action'+keys['name']+'keys').text(keys['keys'])
    });

    for (var i = 0; i < action_names.length; i++) {
        $('#action_'+action_names[i]).click(function(event) {
            socket.emit('keys', event.target.innerText);
        });
    }

    $('#clear').click(function(event) {
        socket.emit('keys', 'clear');
    });

    $('#default').click(function(event) {
        socket.emit('keys', 'default');
    });

    function nmea_config(event) {
        socket.emit('baud', {'in': $('#nmea_in').value(), 'out': $('#nmea_out').value, 'baud': $('#nmea_baud')})
    }

    $('#nmea_in').click(nmea_config);
    $('#nmea_out').click(nmea_config);
    $('#nmea_baud').click(nmea_config);

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
