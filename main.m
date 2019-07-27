classdef bicycleApp < matlab.apps.AppBase

    % Properties that correspond to app components
    properties (Access = public)
        BicycleGimbal               matlab.ui.Figure
        UIAxes                      matlab.ui.control.UIAxes
        SteerSliderLabel            matlab.ui.control.Label
        SteerSlider                 matlab.ui.control.Slider
        ForkLengthcmEditFieldLabel  matlab.ui.control.Label
        ForkLengthcmEditField       matlab.ui.control.NumericEditField
        RollSliderLabel             matlab.ui.control.Label
        RollSlider                  matlab.ui.control.Slider
        YawSliderLabel              matlab.ui.control.Label
        YawSlider                   matlab.ui.control.Slider
        WheelbasecmEditFieldLabel   matlab.ui.control.Label
        WheelbasecmEditField        matlab.ui.control.NumericEditField
        SteerAxisTiltSliderLabel    matlab.ui.control.Label
        SteerAxisTiltSlider         matlab.ui.control.Slider
        WheelRadicmEditFieldLabel   matlab.ui.control.Label
        WheelRadicmEditField        matlab.ui.control.NumericEditField
    end

    properties (Access = public)
        fork_length=0.75;
        trail_angle=pi/10;
        wheelbase=1.03;
        r=0.6858/2;
        delta=0;
        psi=0;
        phi=0;
        xp;
        yp;
        zp;
    end
    methods (Access = private)
        
        function segments = bicycle(app)
            % flat plane
            wb=app.wheelbase;%m
            lamda=app.trail_angle;%rad
            CD=app.fork_length;
            e3=[0;0;1];
            e2=[0;1;0];
            e1=[1;0;0];
            radr=app.r; radf=app.r;  %m
            theta=linspace(0,2*pi,60);%rad
          
       
            r1=rotmat(app,e3,app.psi);r2=rotmat(app,r1*e1,app.phi);rrf=r2*r1;
            r3=rotmat(app,e2,-lamda);
            r4=rotmat(app,r3*(e3),app.delta);
            q2=r4*r3;
            q=rrf;
            rrw=rotmat(app,e2,0);
            
            x=0;
            y=0;
            z=0;
            xr=radr*cos(theta);zr=radr*sin(theta)+radr; yr=0*theta;
            xf=radf*cos(theta)+wb;zf=radf*sin(theta)+radf;yf=0*theta;
            c1=rrw*[xr;yr;zr-radr];
            c1=q*[c1(1,:) ;c1(2,:); c1(3,:)+radr]; c2m=rotmat(app,q2*(e2),0)*q2*[xf-wb;yf;zf-radf];c2=q*[c2m(1,:)+wb;c2m(2,:);c2m(3,:)+radf];
            segments.c1=[c1(1,:)+x ; c1(2,:)+y ; c1(3,:)+z];segments.c2=[c2(1,:)+x ; c2(2,:)+y ; c2(3,:)+z];
            
            %Segment 1 (fork)
            p1s=[wb;0;radf];
            
            p1e=[-sin(lamda)*CD+wb;0;cos(lamda)*CD+radf];
            
            s1=q*[p1s p1e];
            segments.s1=[s1(1,:)+x ; s1(2,:)+y; s1(3,:)+z];
            %Segment 2 (frame)
            
            p2s=[0;0;0+radr];
            
            p2e=[0-sin(lamda)*3/4*CD+wb;0;0+cos(lamda)*3/4*CD+radr];
            
            s2=q*[p2s p2e];
            segments.s2=[s2(1,:)+x ; s2(2,:)+y ; s2(3,:)+z];
            
            %Segment 3 (saddle frame)
            
            p3s=[-sin(lamda)*CD/4+wb/3;0;cos(lamda)*CD/4+radr];
            
            p3e=p3s;
            p3e(3)=p3s(3)+0.3;
            
            s3=q*[p3s p3e];
            segments.s3=[s3(1,:)+x ; s3(2,:)+y ; s3(3,:)+z];
            
            %Segment 4 (handlebar)
            
            p4s=[p1e(1)-0.1;p1e(2)-0.3;p1e(3)];
            
            p4l=[p1e(1);p1e(2)-0.3;p1e(3)];
            
            p4r=[p1e(1); p1e(2)+0.3 ;p1e(3)];
            
            p4e=[p1e(1)-0.1; p1e(2)+0.3 ;p1e(3)];
            
            s4=[p4s p4l p4r p4e];
            s4(3,:)=s4(3,:)-CD*cos(lamda)-radr;
            s4(2,:)=s4(2,:);
            s4(1,:)=s4(1,:)+CD*sin(lamda)-wb;
            s4=q2*s4;
            s4(3,:)=s4(3,:)+CD*cos(lamda)+radr;
            s4(2,:)=s4(2,:);
            s4(1,:)=s4(1,:)-CD*sin(lamda)+wb;
            s4=q*s4;
            segments.s4=[s4(1,:)+x ; s4(2,:)+y ; s4(3,:)+z];
            
            %Segment 5 (saddle)
            
            p5s=[p3e(1)-0.03;p3e(2)-0.07;p3e(3)];
            
            p5m=[p3e(1)+0.15; p3e(2); p3e(3)];
            
            p5e=[p3e(1)-0.03; p3e(2)+0.07; p3e(3)];
            
            s5=q*[p5s p5m p5e];
            segments.s5=[s5(1,:)+x ; s5(2,:)+y ; s5(3,:)+z];
                     
            %Segment 7 ( wheel rods)
             o7r=zeros(3,6);
             p7r=q*[o7r(1,:) ; o7r(2,:)   ;    o7r(3,:)+radr];
             segments.s7r=[p7r(1,:)+x ; p7r(2,:)+y ;    p7r(3,:)+z   ];
             
             p7f=q*[o7r(1,:)+wb ; o7r(2,:)   ;    o7r(3,:)+radr];
             segments.s7f=[p7f(1,:)+x ; p7f(2,:)+y ;    p7f(3,:)+z   ];

        end
        
        function plotBike(app,segments)
            hold(app.UIAxes,'off');
            box(app.UIAxes,'on');
            plot3(app.UIAxes,segments.s1(1,:),segments.s1(2,:),segments.s1(3,:),'b','LineWidth',3)
            hold(app.UIAxes,'on');
            patch(app.UIAxes,segments.c1(1,:),segments.c1(2,:),segments.c1(3,:),'cyan','LineWidth',2)
            patch(app.UIAxes,segments.c2(1,:),segments.c2(2,:),segments.c2(3,:),'cyan','LineWidth',2)
            plot3(app.UIAxes,segments.s2(1,:),segments.s2(2,:),segments.s2(3,:),'b','LineWidth',3)
            plot3(app.UIAxes,segments.s3(1,:),segments.s3(2,:),segments.s3(3,:),'b','LineWidth',3)
            plot3(app.UIAxes,segments.s4(1,:),segments.s4(2,:),segments.s4(3,:),'b','LineWidth',3)
            for jj=10:10:60
                plot3(app.UIAxes,[segments.s7r(1,jj/10) segments.c1(1,jj)],[segments.s7r(2,jj/10) segments.c1(2,jj)],[segments.s7r(3,jj/10) segments.c1(3,jj)],'b','LineWidth',2)
                plot3(app.UIAxes,[segments.s7f(1,jj/10) segments.c2(1,jj)],[segments.s7f(2,jj/10) segments.c2(2,jj)],[segments.s7f(3,jj/10) segments.c2(3,jj)],'b','LineWidth',2)
            end
            patch(app.UIAxes,segments.s5(1,:),segments.s5(2,:),segments.s5(3,:),'cyan','LineWidth',3,'EdgeColor','blue')
            unitVectors(app)
%             app.UIAxes.XLim=[-1 1.5];
%             app.UIAxes.YLim=[-1 1];
%             app.UIAxes.ZLim=[0 1.5];
             axis(app.UIAxes, 'equal');
             axis(app.UIAxes, 'tight');
             app.UIAxes.ZLabel.String='Z';

        end
        
        function rm = rotmat(app,uv,angle)   
            s=[0 -uv(3) uv(2);uv(3) 0 -uv(1);-uv(2) uv(1) 0];
            rm=cos(angle)*eye(3)+(1-cos(angle))*(uv)*uv'+sin(angle)*s;
        end
        
        function unitVectors(app)
            point1 = [0.3,0,0];
            point2 = [0,0.3,0];
            point3 = [0 0 0.3];
            origin = [0,0,0];
            plot3(app.UIAxes,[origin(1) point1(1)],[origin(2) point1(2)],[origin(3) point1(3)],'LineWidth',3);
            plot3(app.UIAxes,[origin(1) point2(1)],[origin(2) point2(2)],[origin(3) point2(3)],'LineWidth',3);
            plot3(app.UIAxes,[origin(1) point3(1)],[origin(2) point3(2)],[origin(3) point3(3)],'LineWidth',3);
            surf(app.UIAxes,app.xp,app.yp,app.zp)
        end
    end
    

    % Callbacks that handle component events
    methods (Access = private)

        % Code that executes after component creation
        function startupFcn(app)
            [app.xp ,app.yp] = meshgrid(-1.5:0.5:1.5); % Generate x and y data
            app.zp = zeros(size(app.xp, 1)); % Generate z data
            segments=bicycle(app);
            plotBike(app,segments);  
        end

        % Value changed function: ForkLengthcmEditField
        function ForkLengthcmEditFieldValueChanged(app, event)
            value = app.ForkLengthcmEditField.Value;
            app.fork_length=value/100;
            segments=bicycle(app);
            plotBike(app,segments);
        end

        % Value changing function: SteerSlider
        function SteerSliderValueChanging(app, event)
            changingValue = event.Value;
            app.delta=changingValue*pi/180;
            segments=bicycle(app);
            plotBike(app,segments);
        end

        % Value changing function: RollSlider
        function RollSliderValueChanging(app, event)
            changingValue = event.Value;
            app.phi=changingValue*pi/180;
            segments=bicycle(app);
            plotBike(app,segments);
        end

        % Value changing function: YawSlider
        function YawSliderValueChanging(app, event)
            changingValue = event.Value;
            app.psi=changingValue*pi/180;
            segments=bicycle(app);
            plotBike(app,segments);            
        end

        % Value changed function: WheelRadicmEditField
        function WheelRadicmEditFieldValueChanged(app, event)
            value = app.WheelRadicmEditField.Value;
            app.r=value/100;
            segments=bicycle(app);
            plotBike(app,segments);
        end

        % Value changing function: SteerAxisTiltSlider
        function SteerAxisTiltSliderValueChanging(app, event)
            changingValue = event.Value;
            app.trail_angle=changingValue*pi/180;
            segments=bicycle(app);
            plotBike(app,segments);
        end

        % Value changed function: WheelbasecmEditField
        function WheelbasecmEditFieldValueChanged(app, event)
            value = app.WheelbasecmEditField.Value;
            app.wheelbase=value/100;
            segments=bicycle(app);
            plotBike(app,segments);
        end
    end

    % Component initialization
    methods (Access = private)

        % Create UIFigure and components
        function createComponents(app)

            % Create BicycleGimbal and hide until all components are created
            app.BicycleGimbal = uifigure('Visible', 'off');
            app.BicycleGimbal.Position = [100 100 671 522];
            app.BicycleGimbal.Name = 'UI Figure';

            % Create UIAxes
            app.UIAxes = uiaxes(app.BicycleGimbal);
            title(app.UIAxes, 'Bicycle')
            xlabel(app.UIAxes, 'X')
            ylabel(app.UIAxes, 'Y')
            app.UIAxes.XGrid = 'on';
            app.UIAxes.YGrid = 'on';
            app.UIAxes.Position = [68 176 503 327];

            % Create SteerSliderLabel
            app.SteerSliderLabel = uilabel(app.BicycleGimbal);
            app.SteerSliderLabel.HorizontalAlignment = 'right';
            app.SteerSliderLabel.Position = [132 136 34 22];
            app.SteerSliderLabel.Text = 'Steer';

            % Create SteerSlider
            app.SteerSlider = uislider(app.BicycleGimbal);
            app.SteerSlider.Limits = [-90 90];
            app.SteerSlider.ValueChangingFcn = createCallbackFcn(app, @SteerSliderValueChanging, true);
            app.SteerSlider.Position = [86 124 125 3];

            % Create ForkLengthcmEditFieldLabel
            app.ForkLengthcmEditFieldLabel = uilabel(app.BicycleGimbal);
            app.ForkLengthcmEditFieldLabel.HorizontalAlignment = 'right';
            app.ForkLengthcmEditFieldLabel.Position = [36 65 97 22];
            app.ForkLengthcmEditFieldLabel.Text = 'Fork Length (cm)';

            % Create ForkLengthcmEditField
            app.ForkLengthcmEditField = uieditfield(app.BicycleGimbal, 'numeric');
            app.ForkLengthcmEditField.ValueChangedFcn = createCallbackFcn(app, @ForkLengthcmEditFieldValueChanged, true);
            app.ForkLengthcmEditField.Position = [70 42 31 22];
            app.ForkLengthcmEditField.Value = 75;

            % Create RollSliderLabel
            app.RollSliderLabel = uilabel(app.BicycleGimbal);
            app.RollSliderLabel.HorizontalAlignment = 'right';
            app.RollSliderLabel.Position = [312 136 30 22];
            app.RollSliderLabel.Text = 'Roll ';

            % Create RollSlider
            app.RollSlider = uislider(app.BicycleGimbal);
            app.RollSlider.Limits = [-90 90];
            app.RollSlider.ValueChangingFcn = createCallbackFcn(app, @RollSliderValueChanging, true);
            app.RollSlider.Position = [268 125 125 3];

            % Create YawSliderLabel
            app.YawSliderLabel = uilabel(app.BicycleGimbal);
            app.YawSliderLabel.HorizontalAlignment = 'right';
            app.YawSliderLabel.Position = [495 136 28 22];
            app.YawSliderLabel.Text = 'Yaw';

            % Create YawSlider
            app.YawSlider = uislider(app.BicycleGimbal);
            app.YawSlider.Limits = [-180 180];
            app.YawSlider.ValueChangingFcn = createCallbackFcn(app, @YawSliderValueChanging, true);
            app.YawSlider.Position = [446 125 125 3];

            % Create WheelbasecmEditFieldLabel
            app.WheelbasecmEditFieldLabel = uilabel(app.BicycleGimbal);
            app.WheelbasecmEditFieldLabel.HorizontalAlignment = 'right';
            app.WheelbasecmEditFieldLabel.Position = [284 62 93 22];
            app.WheelbasecmEditFieldLabel.Text = 'Wheelbase (cm)';

            % Create WheelbasecmEditField
            app.WheelbasecmEditField = uieditfield(app.BicycleGimbal, 'numeric');
            app.WheelbasecmEditField.ValueChangedFcn = createCallbackFcn(app, @WheelbasecmEditFieldValueChanged, true);
            app.WheelbasecmEditField.Position = [312 41 30 22];
            app.WheelbasecmEditField.Value = 103;

            % Create SteerAxisTiltSliderLabel
            app.SteerAxisTiltSliderLabel = uilabel(app.BicycleGimbal);
            app.SteerAxisTiltSliderLabel.HorizontalAlignment = 'right';
            app.SteerAxisTiltSliderLabel.Position = [469 59 79 22];
            app.SteerAxisTiltSliderLabel.Text = 'Steer Axis Tilt';

            % Create SteerAxisTiltSlider
            app.SteerAxisTiltSlider = uislider(app.BicycleGimbal);
            app.SteerAxisTiltSlider.Limits = [-30 30];
            app.SteerAxisTiltSlider.ValueChangingFcn = createCallbackFcn(app, @SteerAxisTiltSliderValueChanging, true);
            app.SteerAxisTiltSlider.Position = [486 52 46 3];

            % Create WheelRadicmEditFieldLabel
            app.WheelRadicmEditFieldLabel = uilabel(app.BicycleGimbal);
            app.WheelRadicmEditFieldLabel.HorizontalAlignment = 'right';
            app.WheelRadicmEditFieldLabel.Position = [165 62 95 22];
            app.WheelRadicmEditFieldLabel.Text = 'Wheel Radi (cm)';

            % Create WheelRadicmEditField
            app.WheelRadicmEditField = uieditfield(app.BicycleGimbal, 'numeric');
            app.WheelRadicmEditField.ValueChangedFcn = createCallbackFcn(app, @WheelRadicmEditFieldValueChanged, true);
            app.WheelRadicmEditField.Position = [196 41 35 20];
            app.WheelRadicmEditField.Value = 34;

            % Show the figure after all components are created
            app.BicycleGimbal.Visible = 'on';
        end
    end

    % App creation and deletion
    methods (Access = public)

        % Construct app
        function app = bicycleApp

            % Create UIFigure and components
            createComponents(app)

            % Register the app with App Designer
            registerApp(app, app.BicycleGimbal)

            % Execute the startup function
            runStartupFcn(app, @startupFcn)

            if nargout == 0
                clear app
            end
        end

        % Code that executes before app deletion
        function delete(app)

            % Delete UIFigure when app is deleted
            delete(app.BicycleGimbal)
        end
    end
end