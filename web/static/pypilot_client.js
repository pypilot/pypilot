/*
#   Copyright (C) 2023 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  
*/


// for now dont translate here
var _ = function(msg) { return msg; }

$(document).ready(function() {
    namespace = '';
    $('#connection').text("N/A");
    $('#values').append("<tr><td>" + _('Name') + "</td><td>" + _("value") + "</td><td>" + _("units") + "</td></tr>");

    // Connect to the Socket.IO server.
    var port = location.port;
    port = pypilot_web_port;
    var socket = io.connect(location.protocol + '//' + document.domain + ':' + port + namespace);

    function pypilot_watch(name, period=true) {
        socket.emit('pypilot', 'watch={"' + name + '":' + JSON.stringify(period) + '}')
    }

    function pypilot_set(name, value) {
        socket.emit('pypilot', name + '=' + JSON.stringify(value));
    }

    var list_values = {};
    socket.on('pypilot_values', function(msg) {
        $('#connection').text('Connected');
        $('#values').empty();

        list_values = JSON.parse(msg);
        var rows = '';
        for (var name in list_values) {
            var info = list_values[name];
            var units = '';
            if('units' in info)
                units = info['units']
            var id = name.split('.').join('_')

            rows += "<tr><td>" + name + "</td><td>";
            var type = info['type'];

            rows += '<div id="value_' + id + '" />';
            if(type == 'RangeProperty') {
                min = info['min'];
                max = info['max'];
                rows += '<input type="range" id="val_' + id + '" min="' + min + '" max="' + max + '" value = "' + 0 + '" step=".0001" style="width: 100%" />';
            } else if(type == 'BooleanProperty') {
                rows += '<input type="checkbox" id="check_' + id + '" />';
            } else if(type == 'EnumProperty') {
                rows += "<select id='val_" + id + "'>";
                var choices = info['choices'];
                for (var choice in choices) {
                    var schoice = choices[choice].toString();
                    rows += '<option value="' + schoice + '">' + schoice + '</option>';
                }
                rows += "</select>";
            } else if(type == 'ResettableValue') {
                rows += '<button id="resetbutton_' + id + '">' + _('Reset') + '</button>';
            }
            rows += "</td><td>" + units + "</td></tr>";

        }
        $('#values').append(rows);

        for (var name in list_values) {
            pypilot_watch(name, 1)
            var info = list_values[name];
            var type = info['type'];
            var id = name.split('.').join('_')
            if(type == 'RangeProperty') {
                $('#val_' + id).change(name, function(event) {
                    pypilot_set(event.data, this.valueAsNumber);
                });
            } else if(type == 'BooleanProperty') {
                $('#check_' + id).change(name, function(event) {
                    pypilot_set(event.data, this.checked);
                });
                $('#value_' + id).hide();
            } else if(type == 'EnumProperty') {
                $('#val_' + id).change(name, function(event) {
                    pypilot_set(event.data, this.value);
                });
                $('#value_' + id).hide();
            } else if(type == 'ResettableValue') {
                $("#resetbutton_" + id).click(name, function(event) {
                    pypilot_set(event.data, 0);
                });
            }
        }
    });

    socket.on('pypilot_disconnect', function() {
        $('#connection').text('Disconnected')
    });

    socket.on('pypilot', function(msg) {
        data = JSON.parse(msg);
        for(var name in data) {
            id = name.split('.').join('_')
            var info = list_values[name];
            var type = info['type'];
            var id = name.split('.').join('_')

            if(type == 'RangeProperty')                
                $('#val_' + id).val(data[name]);
            else if(type == 'EnumProperty')
                $('#val_' + id).val(data[name].toString());
            else if(type == 'BooleanProperty') 
                $('#check_' + id).prop('checked', data[name]);
            var text = data[name].toString();
            var ltext = ''
            for(var i=0; i < text.length; i+=40)
                ltext += text.substr(i, 40)+'\n';
            $('#value_'+id).text(ltext)
        }
    });
});
