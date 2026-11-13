# r402 관측 스트림의 명시 domain/topic

기존C1/RGB 수신기가domain216과preview토픽에고정되어운영노드구성과연결되지않던부분을선택인자로분리한다. 기본216·기존preview토픽·촬영60Hz·선택ROS발행률·C1순서/시각추정/센서STOP은유지한다. 명시runtime토픽으로선택해도실물축교정/프레임재표시/구동출력을만들지않는다.

순수검증: staged/scripts→common scripts/src PYTHONPATH에서 test_sensor_stream_selection.py와기존tests/test_camera_publish_cadence.py·tests/test_hardware_preview_runner.py를한번실행한다. AST/diff에서기존데이터변환·주기·종료본문은동일함을확인했다. 실제생산기비교는이관후최종runtime묶음에포함하고센서별독립재시험은추가하지않는다. frame기대계약은별도r403작업이다.
