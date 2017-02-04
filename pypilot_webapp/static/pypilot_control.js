
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
    $('#power_consumption').text("N/A");
    $('#runtime').text("N/A");

    var gains = ['P', 'I', 'D'];
    for (var i = 0; i<gains.length; i++) {
        var w = $(window).width();

//        var info = list_values['ap/' + gains[i]]
//        var min = info['min']
        //        var max = info['max']
        var min = 0, max = 1;
        $('#gain_container').append(gains[i]+' <input type="number" id="' + gains[i] + '" min="' + min + '" max="' + max + '" value = "' + 0 + '" step=".0001" style="width:'+w/10+'px"> ');
        $('#'+gains[i]).change(function(event) {
            signalk_set('ap/'+this.id, this.valueAsNumber);
            block_polling = 2;
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
        $('#aperrors0').text("");
        $('#aperrors1').text("");

        // control
        watch('ap/enabled')
        watch('ap/mode')
        watch('ap/heading_command')

        // calibration
        watch('imu/alignmentQ');
        watch('imu/alignmentCounter');
        watch('imu/compass_calibration');

        // configuration

        watch('servo/controller')


        poll_signalk()
        block_polling = 0;
    });

    // we poll rather than watch some values to avoid excessive cpu in browser
    function poll_signalk() {
        setTimeout(poll_signalk, 1000)

        if(this.block_polling > 0) {
            block_polling--;
            return;
        }

        get = function(name) {
            socket.emit('signalk', {'name': name, 'method': 'get'});
        }

        //var tab = $('input:radio[name=tabbed]:checked').val();
        var tab = currentTab
        get('ap/heading');
        if(tab == 'Control') {
        } else if(tab == 'Gain') {
            var gains = ['P', 'I', 'D'];
            for (var i = 0; i<gains.length; i++)
                get('ap/' + gains[i]);
        } else if(tab == 'Calibration') {
            get('imu/pitch');
            get('imu/heel');
        } else if(tab == 'Configuration') {
            get('servo/Max Current');
        } else if(tab == 'Statistics') {
            get('servo/Power Consumption');
            get('imu/runtime');
            get('servo/engauged');
        }
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
        if(this.block_polling > 0) {
            return;
        }
        
        if('ap/heading' in msg.data) {
            heading = msg.data['ap/heading']['value'];
            if(heading.toString()=="false")
                $('#aperrors0').text('compass or gyro failure!');
            else
                $('#aperrors0').text('');

            $('#heading').text(Math.round(10*heading)/10);                    
        }
        if('ap/enabled' in msg.data) {
            if(msg.data['ap/enabled']['value']) {
                var w = $(window).width();
                $('#tb_engauged button').css('left', w/12+"px");
                $('#tb_engauged').addClass('toggle-button-selected');
            } else {
                $('#tb_engauged button').css('left', "0px")
                $('#tb_engauged').removeClass('toggle-button-selected');
            }
        }
        if('ap/mode' in msg.data) {
            value = msg.data['ap/mode']['value'];
            $('#mode').val(value);
        }
        var gains = ['P', 'I', 'D'];
        for (var i = 0; i<gains.length; i++)
            if('ap/' + gains[i] in msg.data) {
                data = msg.data['ap/' + gains[i]]
                value = data['value'];
                $('#' + gains[i]).val(value);
                if('min' in data)
                    $('#' + gains[i]).attr('min', data['min'])
                if('max' in data)
                    $('#' + gains[i]).attr('max', data['max'])
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
        if('servo/Power Consumption' in msg.data) {
            value = msg.data['servo/Power Consumption']['value'];
            $('#power_consumption').text(Math.round(1e4*value)/1e4);
        }
        
        if('imu/runtime' in msg.data) {
            value = msg.data['imu/runtime']['value'];
            $('#runtime').text(value);
        }

        if('servo/controller' in msg.data) {
            value = msg.data['servo/controller']['value'];
            if(value == 'none')
                $('#aperrors1').text('no motor controller!');
            else
                $('#aperrors1').text('');
        }
            
    });
    
    signalk_set = function(name, value) {
        socket.emit('signalk', {'name': name, 'method': 'set', 'value': value});
    }

    // Control
    $('.toggle-button').click(function(event) {
        if($(this).hasClass('toggle-button-selected')) {
            signalk_set('ap/enabled', false)
        } else {
            signalk_set('ap/heading_command', heading)
            signalk_set('ap/enabled', true)
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

    $('#mode').change(function(event) { signalk_set('ap/mode', $('#mode').val()); });

    
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
        block_polling = 2;
    });

    // Statistics
    $('#reset_power_consumption').click(function(event) {
        signalk_set('servo/Power Consumption', 0);
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
