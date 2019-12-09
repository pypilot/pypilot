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
    $('#rfcode').text("N/A");
    // Connect to the Socket.IO server.
    var port = location.port;
    port = webapp_port;
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

    socket.on('rfcode', function(code) {
        $('#rfcode').text(code);
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
