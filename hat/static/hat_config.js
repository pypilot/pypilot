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
        show_adc_channels();
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
        n = keys['name'].replace(' ', '_')
        n = n.replace('+', 'plus')
        $('#action'+n+'keys').text(keys['keys'])
    });

    shown_found_rf = false;
    socket.on('found_rf_codes', function(channel) {
        if(shown_found_rf)
            return
        shown_found_rf = true;
        if(confirm('detected remote on channel ' + channel + ' program default codes?')) {
            // remove any rf keys
            // program this channel
            socket.emit('program_rf_codes', channel);
        }
    });

    function program_button(event) {
        if($('#action0').text().length == 0) {
            if(confirm('clear all codes for' + ' "' + event.target.innerText + '" ?'))
                socket.emit('keys', 'clearcodes'+event.target.innerText);
        } else
            socket.emit('keys', event.target.innerText);
    }

    socket.on('profiles', function(profiles) {
        html = '<table>';
        for(var profile of profiles) {
            p = profile.replace(' ', '_')
            html += '<tr><td><button id="action_profile_' +
                p + '">profile ' +
                profile + '</button></td><td><span id="actionprofile_' + p + 'keys"></span></td></tr>';
        }
        html += '</table>';
        $('#action_profiles').html(html);

        for(var profile of profiles) {
            p = profile.replace(' ', '_')
            //$('#action_profile_'+p).unbind('click')
            $('#action_profile_'+p).click(program_button);
        }
    });
    
    for (var i = 0; i < action_names.length; i++)
        $('#action_'+action_names[i]).click(program_button);

    $('#clear').click(function(event) {
        socket.emit('keys', 'clear');
    });

    $('#default').click(function(event) {
        socket.emit('keys', 'default');
    });

    function show_adc_channels() {
        count = parseInt($('#adc_channels').val());
        for (var i = 0; i<3; i++)
            if(i < count)
                $('#adc_channel_'+i).show();
            else
                $('#adc_channel_'+i).hide();
    }        

    $('#adc_channels').change(function(event) {
        show_adc_channels();
        send_adc_channels(event);
    });
    for (var i = 0; i<3; i++)
        $('#adc_channel_' + i + '_select').change(send_adc_channels);
    
    function send_adc_channels(event) {
        adc_channels = []
        count = parseInt($('#adc_channels').val());
        for (var i = 0; i<count; i++)
            adc_channels.push($('#adc_channels_'+i+'_select').val());
        socket.emit('config', {'arduino.adc_channels': adc_channels});
    }        

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

    $('#command').change(function(event) {
        socket.emit('config', {'command': document.getElementById('command').value});
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
