"""별도 RaceOps 프로세스. localhost 임시 포트·시험 토큰만 사용한다."""
import argparse
import json
from pathlib import Path


def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    assert a.output.is_dir()
    from arena_autonomy.race_ops_node import RaceOps
    from arena_vehicle_interface.node_lifecycle import run_node
    def factory():
        node=RaceOps()
        (a.output/'ready.json').write_text(json.dumps(dict(host='127.0.0.1',port=node.server.server_address[1]))+'\n')
        return node
    args=['--ros-args','-p','vehicle_id:=r407_pty_only','-p','host:=127.0.0.1','-p','port:=0',
        '-p','token_path:='+str(a.output/'operator_token.txt'),
        '-p','record_path:='+str(a.output/'record.json'),'-p','log_path:='+str(a.output/'ops.jsonl')]
    run_node(factory,args=args)


if __name__=='__main__':main()
