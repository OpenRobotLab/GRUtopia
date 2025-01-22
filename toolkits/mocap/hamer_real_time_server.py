import argparse

from flask import Flask, jsonify
from hamer_real_time import CameraProcessor


def create_app(video_url):
    app = Flask(__name__)

    processor = CameraProcessor(video_url=video_url)

    @app.route('/start', methods=['GET'])
    def start_service():
        if not processor.running:
            processor.start()
            print('Video streaming started!')
            return jsonify({'status': 'started', 'message': 'Video processing started.'}), 200
        else:
            return jsonify({'status': 'error', 'message': 'Already running.'}), 400

    @app.route('/stop', methods=['GET'])
    def stop_service():
        if processor.running:
            processor.stop()
            print('Video streaming stopped!')
            return jsonify({'status': 'stopped', 'message': 'Video processing stopped.'}), 200
        else:
            return jsonify({'status': 'error', 'message': 'Not running.'}), 400

    @app.route('/results', methods=['GET'])
    def get_results():
        results = processor.get_results()
        if results:
            return jsonify(results), 200
        else:
            return jsonify({'status': 'error', 'message': 'No results available.'}), 200

    return app


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Run Flask service with video processing.')
    parser.add_argument('--video_url', type=str, required=True, help='URL of the video stream to process.')
    args = parser.parse_args()

    app = create_app(video_url=args.video_url)
    app.run(host='0.0.0.0', port=5001)
