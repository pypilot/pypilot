  function openTab(evt, tabName) {
    var i, x, tablinks;
    x = document.getElementsByClassName("tab");
    for (i = 0; i < x.length; i++) {
         x[i].style.display = "none";
    }
    tablinks = document.getElementsByClassName("tablink");
    for (i = 0; i < x.length; i++) {
        tablinks[i].className = tablinks[i].className.replace(" w3-red", "");
    }
    document.getElementById(tabName).style.display = "block";
      evt.currentTarget.firstElementChild.className += " w3-red";

      currentTab = tabName;
  }
  currentTab="Control";

$(document).ready(function() {
    namespace = '';
    $('#ping-pong').text("N/A");
    $('#connection').text('Not Connected');
    $('#compass_calibration').text("N/A");
    $('#amp_hours').text("N/A");
    $('#runtime').text("N/A");

    var gains = ['P', 'I', 'D'];
    for (var i = 0; i<gains.length; i++) {
        var w = $(window).width();

        $('#gain_container').append(gains[i]+' <input type="number" id="' + gains[i] + '" min="0" max="1" value = ".005" step=".001" style="width:'+w/10+'px"> ');

        $('#'+gains[i]).change(function(event) {
            signalk_set('ap/'+gains[i], parseFloat($('#'+gains[i]).val()));
            return false;
        });
    }

    
    // Connect to the Socket.IO server.
    var port = location.port;
    port = pypilot_webapp_port;
    var socket = io.connect(location.protocol + '//' + document.domain + ':' + port + namespace);
    
    function watch(name) {
        socket.emit('signalk', {'name': name, 'method': 'watch'});
    }
    
/*    function unwatch(name) {
        socket.emit('signalk', {'name': name, 'method': 'watch', 'value': '0'});
    }*/
    
    // Event handler for new connections.
    socket.on('signalk_connect', function() {
        $('#connection').text('Connected')

        // control
        watch('ap/mode')
        watch('ap/heading_command')

        // calibration
        watch('imu/alignmentQ');
        watch('imu/alignmentCounter');
        watch('imu/compass_calibration');

        // configuration
        watch('servo/Max Current');

        poll_signalk()
    });

    // we poll rather than watch some values to avoid excessive cpu in browser
    function poll_signalk() {
        get = function(name) {
            socket.emit('signalk', {'name': name, 'method': 'get'});
        }

        //var tab = $('input:radio[name=tabbed]:checked').val();
        var tab = currentTab
        if(tab == 'Control') {
            get('imu/heading_lowpass');
        } else if(tab == 'Gain') {
        } else if(tab == 'Calibration') {
            get('imu/pitch');
            get('imu/heel');
        } else if(tab == 'Configuration') {
            
        } else if(tab == 'Statistics') {
            get('servo/Amp Hours');
            get('imu/runtime');
            get('servo/engauged');

        }
        setTimeout(poll_signalk, 1000)
    }

    
    socket.on('disconnect', function() {
        $('#connection').text('Disconnected')
    });
    
    // Event handler for server sent data.
    socket.on('log', function(msg) {
        $('#log').append(msg + "<br>");
    });
    
    // Interval function that tests message latency by sending a "ping"
    var ping_pong_times = [];
    var start_time;
    window.setInterval(function() {
        start_time = (new Date).getTime();
        socket.emit('ping');
//        $('#log').append("ping" + "<br>");
    }, 3000);
    
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
    
    var heading = 0;
    var heading_command = 0;
    socket.on('signalk', function(msg) {
        if('imu/heading_lowpass' in msg.data) {
            heading = msg.data['imu/heading_lowpass']['value'];
            $('#heading').text(Math.round(10*heading)/10);                    
        }
        if('ap/mode' in msg.data) {
            if(msg.data['ap/mode']['value'] == 'disabled') {
                $('#tb_engauged button').css('left', "0px")
                $('#tb_engauged').removeClass('toggle-button-selected');
            } else {
                var w = $(window).width();
                $('#tb_engauged button').css('left', w/12+"px");
                $('#tb_engauged').addClass('toggle-button-selected');
            }
        }
        if('ap/heading_command' in msg.data) {
            heading_command = msg.data['ap/heading_command']['value'];
            $('#heading_command').text(Math.round(heading_command));
        }
        if('servo/engauged' in msg.data) {
            if(msg.data['servo/engauged']['value'])
                $('#servo_engauged').text('Engauged');
            else
                $('#servo_engauged').text('Disengauged');
        }

        // calibration
        if('imu/alignmentCounter' in msg.data) {
            $('.myBar').width((100-msg.data['imu/alignmentCounter']['value'])+'%');
        }
        if('imu/compass_calibration' in msg.data) {
            value = msg.data['imu/compass_calibration']['value'];
            $('#compass_calibration').val(value);
        }

        // configuration
        if('servo/Max Current' in msg.data) {
            value = msg.data['servo/Max Current']['value'];
            $('#max_current').val(value);
        }

        // statistics
        if('servo/Amp Hours' in msg.data) {
            value = msg.data['servo/Amp Hours']['value'];
            $('#amp_hours').text(Math.round(1e4*value)/1e4);
        }
        
        if('imu/runtime' in msg.data) {
            value = msg.data['imu/runtime']['value'];
            $('#runtime').text(value);
        }
    });
    
    signalk_set = function(name, value) {
        socket.emit('signalk', {'name': name, 'method': 'set', 'value': value});
    }

    // Control
    $('.toggle-button').click(function(event) {
        if($(this).hasClass('toggle-button-selected')) {
            signalk_set('ap/mode', 'disabled')
        } else {
            signalk_set('ap/heading_command', heading)
            signalk_set('ap/mode', 'compass')
        }
    });
    
    move = function(x) {
        var engauged = $('#tb_engauged').hasClass('toggle-button-selected');
        if(engauged) {
            signalk_set('ap/heading_command', heading_command + x)
        } else {
            if(x != 0) {
                setTimeout(function(){
                    signalk_set('servo/command', x/10.0);
                    x -= Math.abs(x)/x;
                }, 100);
            }
        }
    }
    
    $('#port10').click(function(event) { move(-10); });
    $('#port1').click(function(event) { move(-1); });
    $('#star1').click(function(event) { move(1); });
    $('#star10').click(function(event) { move(10); });

    // Gain

    // Calibration
    $('#level').click(function(event) {
        signalk_set('imu/alignmentCounter', 100);
        signalk_set('imu/alignmentType', 'level');
        return false;
    });

    // Configuration
    $('#max_current').change(function(event) {
        signalk_set('servo/Max Current', parseFloat($('#max_current').val()));
        return false;
    });



    openTab("Control");

    function openTab(name) {
        var i;
        var x = document.getElementsByClassName("tabname");
        for (i = 0; i < x.length; i++) {
            x[i].style.display = "none";
        }
        document.getElementById(name).style.display = "block";
    }

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
