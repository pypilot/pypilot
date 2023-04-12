/*
#   Copyright (C) 2023 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  
*/

function openTab(evt, tabName) {
    var i, x, tablinks;
    x = document.getElementsByClassName("tab");
    for (i = 0; i < x.length; i++) {
         x[i].style.display = "none";
    }
    tablinks = document.getElementsByClassName("tablink");
    for (i = 0; i < tablinks.length; i++) {
        tablinks[i].className = tablinks[i].className.replace(" w3-red", "");
        tablinks[i].className = tablinks[i].className.replace(" active", "");
    }

    document.getElementById(tabName).style.display = "block";
    if(evt.currentTarget.firstElementChild) {
        evt.currentTarget.firstElementChild.className += " w3-red";
        evt.currentTarget.firstElementChild.className += " active";
    }
    
    currentTab = tabName;
    setup_watches()
}

var setup_watches = false;
var watches = {};

currentTab="Control";

$(document).ready(function() {
    namespace = '';
    $('#ping-pong').text("N/A");
    $('#connection').text(_('Not Connected'));
    $('#imu_heading').text("N/A");
    $('#pitch').text("N/A");
    $('#roll').text("N/A");
    $('#rudder').text("N/A");
    $('#power_consumption').text("N/A");
    $('#runtime').text("N/A");

    // Connect to the Socket.IO server.
    var port = location.port;
    port = pypilot_web_port;
    var socket = io.connect(location.protocol + '//' + document.domain + ':' + port + namespace);

    function is_touch_enabled() {
        return ( 'ontouchstart' in window ) ||
            ( navigator.maxTouchPoints > 0 ) ||
            ( navigator.msMaxTouchPoints > 0 );
    }

    function show_gains() {
        var x = document.getElementsByClassName('gain');
        for (i = 0; i < x.length; i++)
            x[i].style.display = 'none';

        pilot =$('#pilot').val();
        x = document.getElementsByClassName('pilot'+pilot);
        for (i = 0; i < x.length; i++)
            x[i].style.display = 'table-row';
    }
    
    // Event handler for new connections.
    var servo_command = 0, servo_command_timeout=0;
    var gains = [];
    var conf_names = [];
    var profile="default", profiles = [profile];
    var touch = is_touch_enabled();
    var register = true;
    var rudder_source = 'none';

    socket.on('pypilot_values', function(msg) {
        watches = {};
        var list_values = JSON.parse(msg);
        var handlers = {};

        function MakeRange(name, iname, subname, size, size2) {
            htm = ''
            var info = list_values[name];
            var min = info['min'];
            var max = info['max'];
            htm+='<div style="display: table-cell;font-size: ' + size + ';">'+subname+'</div>';

            if(touch) {
                function button(bname, text, per) {
                    htm += '<div style="display: table-cell;width: 15%; height 20%; border: 10px solid transparent;"><button id="' + iname + bname +'" class="button button-resizable2 font-resizable2">' + text + '</button></div>';
                    range = max - min;
                    step = per * range / 100;
                    handlers[iname + bname] = {step: step, iname: iname, name: name, func: function(event) {
                        cur = Number($('#' + event.data.iname + 'label').text())
                        pypilot_set(event.data.name, cur + event.data.step);
                    }};
                }
                button('1', '<<', -10);
                button('2', '<', -1);
                button('3', '>', 1);
                button('4', '>>', 10);
            }
            
            htm += '<div style="display: table-cell;width: 100%">';
            if(touch) {
                htm += '<div class="progress"><div id="' + iname + '" class="bar" min="' + min + '" max="' + max + '"></div></div>'             
            } else {
                htm += '<input type="range" id="' + iname + '" min="' + min + '" max="' + max + '" value = "' + 0 + '" step=".0001" style="width: 100%"></input>'
                handlers[iname] = {name: name, func: function(event) {
                    pypilot_set(event.data.name, this.valueAsNumber);
                }};
            }
            htm += '</div>'
            htm += '<div id="' + iname + 'label" style="display: table-cell; font-size: '+size2 + ';"></div>';
            return htm;
        }
        
        $('#connection').text(_('Connected'));
        $('#aperrors0').text("");
        $('#aperrors1').text("");

        // control
        pypilot_watch('ap.enabled');
        pypilot_watch('ap.mode');
        pypilot_watch('ap.modes');
        pypilot_watch('ap.tack.timeout', .25);
        pypilot_watch('ap.tack.state');
        pypilot_watch('ap.tack.direction');

        pypilot_watch('ap.heading_command', .5);
        pypilot_watch('rudder.source');

        if(register)
            $('#mode').change(function(event) {
                pypilot_set('ap.mode', $('#mode').val());
            });

        // gain
        pypilot_watch('profile');
        pypilot_watch('profiles');
        pypilot_watch('ap.pilot');
        $('#gain_container').text('');

        if(register) {
            $('#profile').change(function(event) {
                pypilot_set('profile', $('#profile').val());
            });
            
            $('#add_profile').click(function(event) {
                profile = prompt(_("Enter profile name."));
                if(profile != null) {
                    if(profiles.includes(profile))
                        alert("Already have profile " + profile);
                    else {
                        profiles.push(profile);
                        //pypilot_set('profiles', profiles);
                        pypilot_set('profile', profile);
                    }
                }
            });

            $('#remove_profile').click(function(event) {
                if(!confirm(_("Remove current profile?")))
                    return;
                new_profiles = []
                for(var p of profiles)
                    if(p != profile)
                        new_profiles.push(p);
                pypilot_set('profiles', new_profiles);
            });
        }
        
        
        gains = [];
        for (var name in list_values)
            if('AutopilotGain' in list_values[name] && name.substr(0, 3) == 'ap.')
                gains.push(name);

        html = ''
        html += '<div style="display: table; border-collapse: collapse;">';
        for (var i = 0; i<gains.length; i++) {
            var sp = gains[i].split('.');
            var subname = sp[3];
            var pilot = sp[2];
            var iname = 'gain'+pilot+subname
            html+='<div class="gain pilot' + pilot + '" style="display: none; border=10px solid #000;">';
            html += MakeRange(gains[i], iname, subname, '3em', '2em');
            html += '</div>'
        }
        html += '</div>';
        $('#gain_container').append(html);

        $('#gain_container').append('<div class="w3-row">Pilot&emsp;<select id="pilot">');
        if('ap.pilot' in list_values && 'choices' in list_values['ap.pilot']) {
            var pilots = list_values['ap.pilot']['choices'];
            for (var pilot in pilots)
                $('#pilot').append('<option value="' + pilots[pilot] + '">' + pilots[pilot] + '</option>');
        }

        $('#gain_container').append('</select></div>')

        if(register)
        $('#pilot').change(function(event) {
            pypilot_set('ap.pilot', $('#pilot').val());
            show_gains();
        });

        // calibration
        pypilot_watch('imu.heading_offset', .5);
        pypilot_watch('imu.alignmentQ', .5);
        pypilot_watch('imu.alignmentCounter', .25);
        pypilot_watch('imu.compass.calibration.locked');
        pypilot_watch('imu.accel.calibration.locked');

        if(register)
        $('#accel_calibration_locked').change(function(event) {
            check = $('#accel_calibration_locked').prop('checked');
            pypilot_set('imu.accel.calibration.locked', check);
        });

        pypilot_watch('imu.compass.calibration.locked');

        if(register)
        $('#compass_calibration_locked').change(function(event) {
            check = $('#compass_calibration_locked').prop('checked');
            pypilot_set('imu.compass.calibration.locked', check);
        });


        pypilot_watch('rudder.offset');
        pypilot_watch('rudder.scale');
        pypilot_watch('rudder.nonlinearity');
        pypilot_watch('rudder.range');

        // configuration
        $('#configuration_container').text('')
        conf_names = [];
        for (var name in list_values)
            if(list_values[name]['type'] == 'RangeSetting')
                conf_names.push(name); // remove ap.
        conf_names.sort();

        phtml = '';
        $('#configuration_container').append('<div style="display: table; border-collapse: collapse;">');
        
        for (var i = 0; i<conf_names.length; i++) {
            html = '';
            var name = conf_names[i];
            var info = list_values[name];
            var units = info['units'];
            html += '<div style="display: table-row;">';
            html += MakeRange(name, 'confname'+i, name, '1em', '1em');
            html += '<div style="display: table-cell;">' + units + '</div>';
            html += '</div>';
            var persistent = info['profiled'];
            
            if(info['profiled'])
                phtml += html;
            else
                $('#configuration_container').append(html);
        }
        $('#configuration_container').append('</div>');
        $('#configuration_container').append('<br>');
        $('#configuration_container').append(_('The following settings are captured by the current profile'));


        $('#configuration_container').append('<div style="display: table; border-collapse: collapse;">');
        $('#configuration_container').append(phtml);
        $('#configuration_container').append('</div>');


        $('#configuration_container').append('<p>Configure more settings with <a href="/client">pypilot client</a>');

        if(tinypilot)
            $('#configuration_container').append('<p><a href="/wifi">' + _('Configure Wifi') + '</a>');

        url = window.location.href.split('://')
        url = url.length > 1 ? url[1] : url[0];
        
        $('#configuration_container').append('<p><a href="http://' + url.split(':')[0].split('/')[0] + ':33333">' + _('Configure LCD Keypad and Remotes') + '</a>');

        pypilot_watch('nmea.client');

        pypilot_watch('imu.error');
        pypilot_watch('imu.warning');
        pypilot_watch('servo.controller');
        pypilot_watch('servo.flags');

        // install function handlers for configuration sliders/buttons
        if(register) {
            for (var handler in handlers)
                if(touch)
                    $('#' + handler).click(handlers[handler], handlers[handler]['func']);
                else
                    $('#' + handler).change(handlers[handler], handlers[handler]['func']);
        }
        register=false;

        window_resize();        
            
        setup_watches();
    });

    socket.on('pypilot_disconnect', function() {
        $('#connection').text(_('Disconnected'))
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
    
    var heading = 0;
    var heading_command = 0;
    var heading_set_time = new Date().getTime();
    var heading_local_command;
    var last_rudder_data = {};
    var current_mode = "NA";

    function heading_str(heading) {
        if(heading.toString()=="false")
            return "N/A";

        // round to 1 decimal place
        heading = Math.round(10*heading)/10

        if(current_mode == 'wind' || current_mode == 'true wind') {
            if(heading > 0)
               heading += '-';
        }
        return heading;
    }

    socket.on('pypilot', function(msg) {
        data = JSON.parse(msg);

        if('ap.mode' in data) {
            current_mode = data['ap.mode'];
            $('#mode').val(current_mode);
        }

        if('ap.modes' in data) {
            value = data['ap.modes'];
            $('#mode').empty();
            for (let mode of value)
                $('#mode').append('<option value="' + mode + '">' + mode + '</option>');
            $('#mode').val(current_mode);
        }

        if('ap.heading' in data) {
            heading = data['ap.heading'];
            $('#heading').text(heading_str(heading));
        }

        if('rudder.source' in data) {
            if(data['rudder.source'] == 'none')
                $('#center_button').hide();
            else
                $('#center_button').show();
        }

        
        if('ap.tack.timeout' in data)
            $('#tack_timeout').text(Math.round(10*data['ap.tack.timeout'])/10);

        if('ap.tack.state' in data) {
            value = data['ap.tack.state'];
            $('#tack_state').text(value);
            if(value == 'none') {
                $('#tack_button').text(_('Tack'))
                $('#tack_button').attr('state', 'tack')
            } else {
                $('#tack_button').text(_('Cancel'))
                $('#tack_button').attr('state', 'none')
            }
        }
        
        if('ap.enabled' in data) {
            if(data['ap.enabled']) {
                var w = $(window).width();
                $('#tb_engaged button').css('left', w/12+"px");
                $('#tb_engaged').addClass('toggle-button-selected');

                $('#port10').text('10');
                $('#port1').text('1');
                $('#star1').text('1');
                $('#star10').text('10');

                $('#center_span').hide();
                $('#tack_button').show();
            } else {
                $('#tb_engaged button').css('left', "0px");
                $('#tb_engaged').removeClass('toggle-button-selected');
                
                $('#port10').text('<<');
                $('#port1').text('<');
                $('#star1').text('>');
                $('#star10').text('>>');

                $('#center_span').show();
                $('#tack_button').hide();
            }
        }

        if('ap.heading_command' in data) {
            heading_command = data['ap.heading_command'];
            $('#heading_command').text(heading_str(heading_command));
        }

        if('ap.pilot' in data) {
            value= data['ap.pilot'];
            $('#pilot').val(value);
            show_gains();
        }

        if('profile' in data) {
            profile = data['profile'].toString();
            $('#profile').val(profile);
        }

        if('profiles' in data) {
            profiles = data['profiles'];
            $('#profile').empty();
            for (p of profiles)
                $('#profile').append('<option value="' + p + '">' + p + '</option>');
            $('#profile').val(profile);
        }

        
        for (var i = 0; i<gains.length; i++)
            if(gains[i] in data) {
                value = data[gains[i]];
                var sp = gains[i].split('.');
                var subname = sp[3];
                var pilot = sp[2];
                var iname = 'gain'+pilot+subname
                if(value != $('#' + iname).valueAsNumber) {
                    if(touch) {
                        min = Number($('#' + iname).attr('min'));
                        max = Number($('#' + iname).attr('max'));
                        per = 100*(value-min)/(max-min);
                        $('#' + iname).width(per+'%');
                    } else // slider
                        $('#' + iname).val(value);
                    $('#' + iname + 'label').text(value);
                }
            }

        if('servo.engaged' in data) {
            if(data['servo.engaged'])
                $('#servo_engaged').text(_('Engaged'));
            else
                $('#servo_engaged').text(_('Disengaged'));
        }

        // calibration
        if('imu.heading' in data)
            $('#imu_heading').text(data['imu.heading']);
        if('imu.pitch' in data)
            $('#pitch').text(data['imu.pitch']);
        if('imu.roll' in data)
            $('#roll').text(data['imu.roll']);
        if('imu.alignmentCounter' in data)
            $('#levelprogress').width((100-data['imu.alignmentCounter'])+'%');
        if('gps.alignmentCounter' in data)
            $('#align_compassprogress').width((100-10*data['gps.alignmentCounter'])+'%');
        if('imu.heading_offset' in data)
            $('#imu_heading_offset').val(data['imu.heading_offset']);
        if('imu.accel.calibration.locked' in data)
            $('#accel_calibration_locked').prop('checked', data['imu.accel.calibration.locked']);
        if('imu.compass.calibration.locked' in data)
            $('#compass_calibration_locked').prop('checked', data['imu.compass.calibration.locked']);

        var rudder_dict = {'rudder.offset': 'Offset',
                           'rudder.scale': 'Scale',
                           'rudder.nonlinearity': 'Non Linearity'};
        if('rudder.angle' in data) {
            $('#rudder').text(data['rudder.angle']);
            $('#rudder').append('<p>')
            for(var name in rudder_dict)
                if(name in last_rudder_data)
                    $('#rudder').append(' ' + rudder_dict[name] + ' ' + Math.round(100*last_rudder_data[name])/100);
        }

        for(var name in rudder_dict)
            if(name in data)
                last_rudder_data[name] = data[name];

        if('rudder.range' in data)
            $('#rudder_range').val(data['rudder.range']);

        // configuration
        for(i=0; i < conf_names.length; i++) {
            if(conf_names[i] in data) {
                value = data[conf_names[i]];
                var iname = 'confname'+i;
                if(touch) {
                    min = Number($('#' + iname).attr('min'));
                    max = Number($('#' + iname).attr('max'));
                    per = 100*(value-min)/(max-min);
                    $('#' + iname).width(per+'%');
                } else // slider
                    $('#' + iname).val(value);
                $('#' + iname + 'label').text(value);
            }
        }

        if('nmea.client' in data)
            $('#nmea_client').val(data['nmea.client']);

        // statistics
        if('servo.amp_hours' in data) {
            value = data['servo.amp_hours'];
            $('#amp_hours').text(Math.round(1e4*value)/1e4);
        }

        if('servo.voltage' in data) {
            value = data['servo.voltage'];
            $('#voltage').text(Math.round(1e3*value)/1e3);
        }
        
        if('servo.controller_temp' in data) {
            value = data['servo.controller_temp'];
            $('#controller_temp').text(value);
        }

        if('servo.motor_temp' in data) {
            value = data['servo.motor_temp'];
            $('#motor_temp').text('<br>'+_('Motor temperature')+' '+value.toString()+' C');
        }

        if('ap.runtime' in data) {
            value = data['ap.runtime'];
            $('#runtime').text(value);
        }

        if('ap.version' in data) {
            value = data['ap.version'];
            $('#version').text(value);
        }

        if('imu.error' in data)
            $('#aperrors0').text(data['imu.error']);
        
        if('imu.warning' in data)
            $('#imu_warning').text(data['imu.warning']);

        if('servo.controller' in data) {
            value = data['servo.controller'];
            if(value == 'none')
                $('#aperrors1').text(_('no motor controller!'));
            else
                $('#aperrors1').text('');
        }

        if('servo.flags' in data)
            $('#servoflags').text(data['servo.flags']);
    });
    
    function pypilot_set(name, value) {
        socket.emit('pypilot', name + '=' + JSON.stringify(value));
    }

    function pypilot_watch(name, period=true) {
        if(period === false && !(name in watches && watches[name]))
            return; // already not watching, no need to inform server
        watches[name] = period;
        socket.emit('pypilot', 'watch={"' + name + '":' + JSON.stringify(period) + '}')
    }

    // Control
    $('.toggle-button').click(function(event) {
        if($(this).hasClass('toggle-button-selected')) {
            pypilot_set('ap.enabled', false);
        } else {
            pypilot_set('ap.heading_command', heading);
            pypilot_set('ap.enabled', true);
        }
    });

    $('#center_button').click(function(event) {
        pypilot_set('servo.position', 0);
    });
    
    function move(x) {
        var engaged = $('#tb_engaged').hasClass('toggle-button-selected');
        if(!engaged)
            return;
        
        time = new Date().getTime();
        if(time - heading_set_time > 1000)
            heading_local_command = heading_command;
        heading_set_time = time;
        heading_local_command += x;
        pypilot_set('ap.heading_command', heading_local_command);
    }

    // update manual servo command
    function poll_pypilot() {
        if(servo_command_timeout > 0) {
            servo_command_timeout--;
            if(servo_command_timeout > 0)
                setTimeout(poll_pypilot, 50)
            else    
                servo_command = 0;
            pypilot_set('servo.command', servo_command);
        }
    }

    function mousedown(amount) {
        var engaged = $('#tb_engaged').hasClass('toggle-button-selected');
        if(engaged) {
            servo_command_timeout = 0;            
            return;
        }
        servo_command = amount;
        servo_command_timeout = 120;
        pypilot_set('servo.command', servo_command);
        setTimeout(poll_pypilot, 50)
    }

    function mouseup(event) {
        var engaged = $('#tb_engaged').hasClass('toggle-button-selected');
        if(engaged) {
            servo_command_timeout = 0;            
            return;
        }
        
        servo_command_timeout -= 116;
        if(servo_command_timeout <= 0) {
            servo_command_timeout = 0;
            servo_command = 0;
            pypilot_set('servo.command', 0);
        }
    }

    function nocontext(event) {
        event.preventDefault();
        event.stopPropagation();
        return false;
    }

    buttons = {'#port1': -.6, '#star1': .6, '#port10': -1, '#star10': 1};
    for (var name in buttons) {
        $(name).on('touchstart', nocontext);
        $(name).on('touchmove', nocontext);
        $(name).on('touchend', nocontext);
        $(name).on('touchcancel', nocontext);
        $(name).on('contextmenu', nocontext);
        $(name).on('pointerup', mouseup);
        $(name).on('pointerdown', function(event) {
            mousedown(buttons['#'+event.target.id]);
        });
    }

    $('#port10').click(function(event) { move(-10); });
    $('#port1').click(function(event) { move(-1); });
    $('#star1').click(function(event) { move(1); });
    $('#star10').click(function(event) { move(10); });

    $('#tack_button').click(function(event) {
        if($('#tack_button').attr('state') == 'tack')
            openTab(event, 'Tack');
        else
            pypilot_set('ap.tack.state', 'none');
    });

    // Gain

    // Calibration
    $('#level').click(function(event) {
        pypilot_set('imu.alignmentCounter', 100);
        return false;
    });

    $('#align_compass').click(function(event) {
        pypilot_set('gps.alignmentCounter', 10);
        return false;
    });
    
    $('#imu_heading_offset').change(function(event) {
        pypilot_set('imu.heading_offset', $('#imu_heading_offset').val());
    });

    $('#rudder_centered').click(function(event) {
        pypilot_set('rudder.calibration_state', 'centered');
    });

    $('#rudder_port_range').click(function(event) {
        pypilot_set('rudder.calibration_state', 'port range');
    });

    $('#rudder_starboard_range').click(function(event) {
        pypilot_set('rudder.calibration_state', 'starboard range');
    });

    $('#rudder_reset').click(function(event) {
        pypilot_set('rudder.calibration_state', 'reset');
    });
    
    $('#rudder_range').change(function(event) {
        pypilot_set('rudder.range', $('#rudder_range').val());
    });

    // Configuration
    $('#nmea_client').change(function(event) {
        pypilot_set('nmea.client', $('#nmea_client').val());
    });

    // Tack
    function tack(event, state) {
        pypilot_set('ap.tack.state', state);
        openTab(event, 'Control');
    }
    
    $('#tack_cancel').click(function(event) { tack(event, 'none'); });
    $('#tack_port').click(function(event) {
        pypilot_set('ap.tack.direction', 'port');
        tack(event, 'begin');
    });
    $('#tack_starboard').click(function(event) {
        pypilot_set('ap.tack.direction', 'starboard');
        tack(event, 'begin');
    });

    
    // hack to get to :33333
    document.addEventListener('click', function(event) {
        var target = event.target;
        if (target.tagName.toLowerCase() == 'a')
        {
            if(target.getAttribute('href').match('33333')) {
                target.href = window.location.origin;
                target.port = 33333;
            }

        }
    }, false);

    // Statistics
    $('#reset_amp_hours').click(function(event) {
        pypilot_set('servo.amp_hours', 0);
        return false;
    });

    // openInitialTab
    var i;
    var x = document.getElementsByClassName("tab");
    for (i = 0; i < x.length; i++)
        x[i].style.display = "none";
    document.getElementById(currentTab).style.display = "block";

    function pypilot_watches(names, watch, period) {
        for(var i=0; i< names.length; i++)
            pypilot_watch(names[i], watch ? period : false);
    }
    
    // should be called if tab changes
    setup_watches = function() {
        var tab = currentTab;
        pypilot_watches(['ap.heading', 'rudder.source'], tab == 'Control', 0.5);
        pypilot_watches(gains, tab == 'Gain', 1);
        pypilot_watches(['imu.heading', 'imu.pitch', 'imu.roll', 'rudder.angle'], tab == 'Calibration', .5);
        pypilot_watches(conf_names, tab == 'Configuration', 1);
        pypilot_watches(['servo.amp_hours', 'servo.voltage', 'servo.controller_temp', 'ap.runtime', 'ap.version', 'servo.engaged'], tab == 'Statistics', 1);
    }
    setup_watches();

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
        $(".button-resizable3").each(function(i, obj) {
            $(this).css('width', w/4+"px")
            $(this).css('height', w/8+"px")
        });
        $(".toggle-button-selected button").each(function(i, obj) {
            $(this).css('left', w/12+"px")
        });
    }

    // Set the theme on page load
    setTheme( getCookie('theme') )

    // Bind setTheme action when the user change the theme via the radio buttons
    $('.theme_option').on('click', function(){ setTheme() })


    for (let l of languages)
        $('#language').append('<option value="' + l + '">' + l + '</option>');
    $('#language').val(language);

    $('#language').change(function(event) {
        socket.emit('language', $('#language').val());
    });
    
    $(window).resize(window_resize);
    window_resize();
});

/**
 * When called, setTheme will read selected theme from fround .theme_option and change the theme.
 * 
 * @function setTheme
 * @param themeName {null|string} the name of the theme to apply or null value to read input.theme_option selection (default is null)
 * @return {string} The applyed theme name.
 *
 */
function setTheme( themeName=null ){

    if(themeName==null){
        themeName = $('input.theme_option:checked').val()
    }else{
        $('input.theme_option:checked').prop("checked", false);
        $("input.theme_option[value="+themeName+"]" ).prop("checked", true );
    }

    $('body').attr('theme', themeName )
    setCookie('theme', themeName)

    // The w3 framework use !important 213 times. So it's impossible to cascade over it. Therefor it must be disabled.
    if(themeName == 'dark'){
        $("link[href*='w3.css']").prop('disabled', true);
    }else{
        $("link[href*='w3.css']").prop('disabled', false);
    }

    return themeName
}

/**
 * Create or edit a cookie.
 * 
 * @function setCookie
 * @param key {string} The name of the cookie to set/edit.
 * @param value {string} The value you want to store.
 * @param expiry {int|number|null} The time to live of the cookie in days (default value is 365 days).
 * @return {void} This function does not return anything.
 *
 */
function setCookie(key, value, expiry=365) {
    var expires = new Date();
    expires.setTime(expires.getTime() + (expiry * 24 * 60 * 60 * 1000));
    document.cookie = key + '=' + value + ';expires=' + expires.toUTCString();
}

/**
 * Get the value of a cookie.
 * 
 * @function getCookie
 * @param key {string} the name of the cookie to query.
 * @return {string|null} Return the value of the cookie or null if the cooky is not found.
 *
 */
function getCookie(key) {
    var keyValue = document.cookie.match('(^|;) ?' + key + '=([^;]*)(;|$)');
    return keyValue ? keyValue[2] : 'dark';  // dark by default
}
