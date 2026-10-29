import unittest
from contact_magnitudes import vectors


class ContactMagnitudeTests(unittest.TestCase):
    def contact(self, sign):
        return dict(normal=[dict(z=sign)],wrench=[dict(body1Wrench=dict(force=dict(z=5*sign,x=2*sign)))])

    def test_original_pair_order_does_not_change_magnitude(self):
        a,b=vectors(self.contact(1))[0],vectors(self.contact(-1))[0]
        self.assertEqual(a['normal_force_magnitude_n'],5)
        self.assertEqual(a['normal_force_magnitude_n'],b['normal_force_magnitude_n'])
        self.assertEqual(a['absolute_normal_z'],b['absolute_normal_z'])
        self.assertNotEqual(a['raw_force_z_n'],b['raw_force_z_n'])

    def test_missing_force_is_not_zero(self):
        c=self.contact(1);c['wrench'][0]['body1Wrench']={}
        self.assertIsNone(vectors(c))

    def test_explicit_zero_is_retained(self):
        c=self.contact(1);c['wrench'][0]['body1Wrench']['force']={}
        self.assertEqual(vectors(c)[0]['normal_force_magnitude_n'],0)

    def test_bad_normal_rejected(self):
        c=self.contact(1);c['normal'][0]['z']=.5
        with self.assertRaises(ValueError):vectors(c)


if __name__=='__main__':unittest.main()
