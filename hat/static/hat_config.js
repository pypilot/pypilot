/*
#   Copyright (C) 2021 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  
*/

$(document).ready(function() {
    namespace = '';
    $('#code').text('N/A');
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

    socket.on('voltage', function(msg) {
        $('#voltage').text(msg);
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

    socket.on('profiles', function(profiles) {
        for(var i =0; ; i++) {
            id = $('#action_profile'+String(i))
            if(id.length == 0)
                break;
            if(i< profiles.length)
                id.text('profile ' + profiles[i]);
            else
                id.text('profile ' + String(i));
        }
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

    function config_ir() {
        socket.emit('config', {'pi.ir': document.getElementById('pi_ir').checked});
        socket.emit('config', {'arduino.ir': document.getElementById('arduino_ir').checked});
    }

    $('#pi_ir').click(config_ir);
    $('#arduino_ir').click(config_ir);
    
    $('#arduino_nmea_in').click(function(event) {
        socket.emit('config', {'arduino.nmea.in': document.getElementById('arduino_nmea_in').checked});
    });
    
    $('#arduino_nmea_out').click(function(event) {
        socket.emit('config', {'arduino.nmea.out': document.getElementById('arduino_nmea_out').checked});
    });

    $('#arduino_nmea_baud').click(function(event) {
        socket.emit('config', {'arduino.nmea.baud': document.getElementById('arduino_nmea_baud').value});
    });

    $('#remote').click(function(event) {
        if(!document.getElementById('remote').checked)
            document.getElementById('host').value = 'localhost'
        socket.emit('config', {'host': document.getElementById('host').value});
    });
    
    // Interval function that tests message latency by sending a "ping"
    var ping_pong_times = [];

    var start_time;
    window.setInterval(function() {
        start_time = (new Date).getTime();
        socket.emit('ping');
        //        $('#log').append("ping" + "<br>");
        $('#ping-pong').text('N/A');
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
