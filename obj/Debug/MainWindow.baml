   M S B A M L   `   `   `  ����  DepthBasics-WPF5    /Microsoft.Samples.Kinect.DepthBasics.MainWindow   5   	   W SPresentationCore, Version=4.0.0.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35R NWindowsBase, Version=4.0.0.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35\ XPresentationFramework, Version=4.0.0.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35^ ZWindowsFormsIntegration, Version=4.0.0.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35F 9http://schemas.microsoft.com/winfx/2006/xaml/presentation     5   	   R NSystem.Xaml, Version=4.0.0.0, Culture=neutral, PublicKeyToken=b77a5c561934e089:x,http://schemas.microsoft.com/winfx/2006/xaml     5   	   -   5          � Title$  Depth Basics��5   	   $	��650��5   	   $	��800��6    � 	Resources 5         MediumGreyBrush �� Color  KinectPurpleBrush  KinectBlueBrush �� 
TargetType �� Property �� Value  ScreenshotButton �� 
TargetType �� Orientation ��Name	 �� TextAlignment
 ��Name �� Triggers P� Property P� Value �� 
TargetName��%a  &	        &	 #     &	 F     '�� i     &	 �     7  7  �� 5	   
   $ 	#ff6e6e6e��62   �� 5
   
   $ 	#ff52318f��64   �� 5   
   $ 	#ff00BCF2��62   �� 5   
   " ��6   .�5      ��  �@J�SnapsToDevicePixels . 64   5      �� 5   
   " ��6   .�5      ��  � �� 5      �� 5      " ��6&   .R�5      � .+�5      �� $ 
Horizontal=�6)   
������ .+�5   "   �� $ ScreenshotText��6,   #��z��6D   $
	 Left=�6e   $��Center=�6{   8��  $��15(�6�   � 5   "   $��9,0,0,0q�6'   .+�5   &   �� $
 ScreenshotNormal��6,   $!��Images\ScreenshotNormal.png��6F   $
��None=�6k   $��Center=�6z   �� 5   &   $
 ScreenshotHover��6,   $ ��Images\ScreenshotHover.png��6E   $
��None=�6i   $��Center=�6x   $y�	Collapsed=�6�   5   #   5      5       5      P� 5       �@J�IsMouseOver . . �5    "   �� $ ScreenshotNormal��6Q    � y�$ 	Collapsed=�6?   6Q   �� 5!   "   $ ScreenshotHover��6O    � y�$ Visible=�6?   6O   �� 5"   "   8  $ ScreenshotText��6h    � ��5#      5%      5'      5(      5)      .��5*      � $��	10 0 10 0q�6   $
��auto��6    � RowDefinitions 5+   
   �� 5,      $
��Auto �6   �� 5-      $��* �6   �� 5.      $
��Auto �6   �� 5/      $
��Auto �6    � ColumnDefinitions 51   
   �� 52      $	��200 �6   6-   �� 53      $
��auto �6   6/   .+�55   
   (� -   5        $��orginal��55      $��1��6!   $��0��6.   $	��200��6>   .��56      �� $��UniformToFill=�64   ��6   �� *ImageSource1+64   57      (� 58   
   $��1��6   $��1��6    $��10 10 10 10q�60   .��59      � .+�5:      �� -   5        $��camera��5:      �� 5;      -   5        $��canvas��5;      �� 5<      $��UniformToFill=�67   ��6   �� *ImageSource+67   5=      5>      �� 5@   
   $��	0,0,0,561q�6   $��2��6$   .+�5A      �� -   5        $
 text��5A      $	��123��6&   $��78��63   $�� ��6>   $	��100(�6F   
���   � �� Right$ 0��6i    �� Left$ 72��6z   
 �� Top$	 388��6�    J� RenderTransformOrigin$ 
0.615,0.32&�6�   5B      � 5D   
   $��2��6   $��1��6   $��10 0 0 0q�6,    5E      �� 5F      $
��Auto �6     5H      �� 5I      $
��auto �6#   62   �� 5J      $
��auto �6#   62   �� 5K      $
��auto �6#   62   �� 5L      $
��auto �6#   62   �� 5M      $
��auto �6#   62   �� 5N      $
��auto �6#   62   �� 5O      $
��auto �6#   62   5R      �� 5T   
   -   5        $��	statusBar��5T   ?   $��3��6   $��Stretch=�6!   $��Bottom=�6P   
�������#��[  6~   $��6��6�   .@�5U      �� ��6   �� *
StatusText+5V      5W      5Y      