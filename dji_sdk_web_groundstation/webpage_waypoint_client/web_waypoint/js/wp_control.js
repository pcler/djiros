
var dji_home= new BMap.Point(113.958004, 22.542494);
var lastMkr = new BMap.Marker(dji_home);

$( document ).ready(function() {
	my_socket = new WebSocket("ws://localhost:19871");
	my_Communicator= new Communicator(my_socket);

	// create map
	map = new BMap.Map("allmap");
	map.centerAndZoom(dji_home, 15);
	map.enableScrollWheelZoom();
	map.disableDoubleClickZoom();
	map.enableKeyboard();
	// create planner
	my_planner = new Planner(map);
	my_planner.reset();
        // create global position displayer
        var icon = new BMap.Icon('http://icons.iconarchive.com/icons/custom-icon-design/flatastic-6/16/Circle-icon.png', new BMap.Size(16,16), {
            anchor : new BMap.Size(8, 8)
        });
        // update global position per second
        var globalPosTimer = setInterval(function() {
            map.removeOverlay(lastMkr);
            var globalPos = my_Communicator.getGlobalPos();
            var mkr = new BMap.Marker(new BMap.Point(globalPos[1], globalPos[0]), {
                icon : icon
            });
            map.addOverlay(mkr);
            lastMkr = mkr;
        }, 1000);

	//bind events to control panel wadgets
	$( "#start-plan" ).bind( "click", function() {

		//my_Communicator.getGlobalPosition();
                var globalPos = my_Communicator.getGlobalPos();
		console.log('Home position: ' + globalPos[0] + ', ' + globalPos[1]);
		if(globalPos[0] == 0 || globalPos[1] == 0){
			alert("Home Location Not Recorded!");
			return;
		}
			
		var drone_home = new BMap.Point(globalPos[1], globalPos[0]);
		map.setCenter(drone_home);

		if (my_planner.isStandby())
		{
			my_planner.addFirstMarker(drone_home);
			my_planner.enableMapClick();
		}
		else if (my_planner.isFinished())
		{
			my_planner.continuePlanning();
		}
	});

	$("#confirm-mission").bind("click", function() {
		my_planner.confirmMission();
	});

        $("#reset-mission").bind("click", function() {
		my_planner.reset();
                $("#monitor").empty();
	});

	$("#upload-mission").bind("click", function() {
		if (typeof(my_Communicator) == 'undefined')
			alert("Drone not Connected!");
		else
            my_Communicator.uploadWayline();
	});

	$("#open-navmode").bind("click", function() {
		if (typeof(my_Communicator) == 'undefined')
			alert("Drone not Connected!");
		else
            my_Communicator.setNavigationMode();
	});

	$("#start-mission").bind("click",function() {
		if (typeof(my_Communicator) == 'undefined')
			alert("Drone not Connected!");
		else
            my_Communicator.startWayline();

	});

	$("#pause-mission").bind("click",function() {
		if (typeof(my_Communicator) == 'undefined')
			alert("Drone not Connected!");
		else
            my_Communicator.pauseWayline();

	});
	$("#resume-mission").bind("click",function() {
		if (typeof(my_Communicator) == 'undefined')
			alert("Drone not Connected!");
		else
            my_Communicator.continueWayline();

	});

	$("#cancel-mission").bind("click",function() {
		if (typeof(my_Communicator) == 'undefined')
			alert("Drone not Connected!");
		else
			my_Communicator.cancelWayline();

	});

	$("#close-navmode").bind("click",function() {
		if (typeof(my_Communicator) == 'undefined')
			alert("Drone not Connected!");
		else
			my_Communicator.stopNavigationMode();

	});
});

